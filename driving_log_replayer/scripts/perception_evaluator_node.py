#!/usr/bin/env python3

# Copyright (c) 2021 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import os
from pathlib import Path
from typing import Dict
from typing import List
from typing import Tuple
from typing import Union

from autoware_auto_perception_msgs.msg import DetectedObject
from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import TrackedObject
from autoware_auto_perception_msgs.msg import TrackedObjects
from geometry_msgs.msg import TransformStamped
from perception_eval.common.object import DynamicObject
from perception_eval.common.schema import FrameID
from perception_eval.common.shape import Shape
from perception_eval.common.shape import ShapeType
from perception_eval.common.status import get_scene_rates
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation import get_object_status
from perception_eval.evaluation import PerceptionFrameResult
from perception_eval.evaluation.metrics import MetricsScore
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer3D
from perception_eval.util.logger_config import configure_logger
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import MarkerArray
import yaml

from driving_log_replayer.node_common import transform_stamped_with_euler_angle
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.result import PickleWriter
from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter


def get_label(classification: ObjectClassification) -> str:
    if classification.label == ObjectClassification.UNKNOWN:
        return "unknown"
    elif classification.label == ObjectClassification.CAR:
        return "car"
    elif classification.label == ObjectClassification.TRUCK:
        return "truck"
    elif classification.label == ObjectClassification.BUS:
        return "bus"
    elif classification.label == ObjectClassification.TRAILER:
        # not implemented in iv
        return "trailer"
    elif classification.label == ObjectClassification.MOTORCYCLE:
        # iv: motorbike, auto: motorbike
        return "motorbike"
    elif classification.label == ObjectClassification.BICYCLE:
        return "bicycle"
    elif classification.label == ObjectClassification.PEDESTRIAN:
        return "pedestrian"
    # not implemented in auto
    # elif classification.label == ObjectClassification.ANIMAL:
    #     return "animal"
    else:
        return "other"


def get_most_probable_classification(
    array_classification: List[ObjectClassification],
) -> ObjectClassification:
    highest_probability = 0.0
    highest_classification = None
    for classification in array_classification:
        if classification.probability >= highest_probability:
            highest_probability = classification.probability
            highest_classification = classification
    return highest_classification


class PerceptionResult(ResultBase):
    def __init__(self, condition: Dict):
        super().__init__()
        self.__pass_rate = condition["PassRate"]
        self.__success = 0
        self.__total = 0

    def update(self):
        test_rate = 0.0 if self.__total == 0 else self.__success / self.__total * 100.0
        success = test_rate >= self.__pass_rate
        summary_str = f"{self.__success} / {self.__total } -> {test_rate:.2f}%"

        if success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def add_frame(
        self, frame: PerceptionFrameResult, skip: int, header: Header, map_to_baselink: Dict
    ) -> Tuple[MarkerArray, MarkerArray]:
        self.__total += 1
        has_objects = True
        if (
            frame.pass_fail_result.tp_object_results == []
            and frame.pass_fail_result.fp_object_results == []
            and frame.pass_fail_result.fn_objects == []
        ):
            has_objects = False

        success = (
            "Success"
            if frame.pass_fail_result.get_fail_object_num() == 0 and has_objects
            else "Fail"
        )
        if success == "Success":
            self.__success += 1
        out_frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        out_frame["PassFail"] = {
            "Result": success,
            "Info": [
                {
                    "TP": len(frame.pass_fail_result.tp_object_results),
                    "FP": len(frame.pass_fail_result.fp_object_results),
                    "FN": len(frame.pass_fail_result.fn_objects),
                }
            ],
        }
        marker_ground_truth = MarkerArray()
        color_success = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)

        for cnt, obj in enumerate(frame.frame_ground_truth.objects, start=1):
            bbox, uuid = eval_conversions.object_state_to_ros_box_and_uuid(
                obj.state, header, "ground_truth", cnt, color_success, obj.uuid
            )
            marker_ground_truth.markers.append(bbox)
            marker_ground_truth.markers.append(uuid)

        marker_results = eval_conversions.pass_fail_result_to_ros_points_array(
            frame.pass_fail_result, header
        )

        self._frame = out_frame
        self.update()
        return marker_ground_truth, marker_results

    def add_final_metrics(self, final_metrics: Dict):
        self._frame = {"FinalScore": final_metrics}


class PerceptionEvaluator(Node):
    def __init__(self):
        super().__init__("perception_evaluator")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("result_json_path", "")
        self.declare_parameter("t4_dataset_path", "")
        self.declare_parameter("result_archive_path", "")

        self.__timer_group = MutuallyExclusiveCallbackGroup()
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self, spin_thread=True)

        scenario_path = os.path.expandvars(
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )
        self.__scenario_yaml_obj = None
        with open(scenario_path, "r") as scenario_file:
            self.__scenario_yaml_obj = yaml.safe_load(scenario_file)
        self.__result_json_path = os.path.expandvars(
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )
        result_archive_path = Path(
            os.path.expandvars(
                self.get_parameter("result_archive_path").get_parameter_value().string_value
            )
        )
        result_archive_path.mkdir(exist_ok=True)

        self.__pkl_path = result_archive_path.joinpath("scene_result.pkl").as_posix()
        self.__t4_dataset_paths = [
            os.path.expandvars(
                self.get_parameter("t4_dataset_path").get_parameter_value().string_value
            )
        ]
        self.__perception_eval_log_path = result_archive_path.parent.joinpath(
            "perception_eval_log"
        ).as_posix()

        try:
            self.__condition = self.__scenario_yaml_obj["Evaluation"]["Conditions"]
            self.__result = PerceptionResult(self.__condition)

            self.__result_writer = ResultWriter(
                self.__result_json_path, self.get_clock(), self.__condition
            )

            p_cfg = self.__scenario_yaml_obj["Evaluation"]["PerceptionEvaluationConfig"]
            c_cfg = self.__scenario_yaml_obj["Evaluation"]["CriticalObjectFilterConfig"]
            f_cfg = self.__scenario_yaml_obj["Evaluation"]["PerceptionPassFailConfig"]

            self.__evaluation_task = p_cfg["evaluation_config_dict"]["evaluation_task"]
            p_cfg["evaluation_config_dict"][
                "label_prefix"
            ] = "autoware"  # Add a fixed value setting

            frame_id, msg_type, topic_ns = self.get_frame_id_and_msg_type()
            self.__frame_id = FrameID.from_value(frame_id)

            evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
                dataset_paths=self.__t4_dataset_paths,
                frame_id=frame_id,
                result_root_directory=os.path.join(
                    self.__perception_eval_log_path, "result", "{TIME}"
                ),
                evaluation_config_dict=p_cfg["evaluation_config_dict"],
                load_raw_data=False,
            )
            _ = configure_logger(
                log_file_directory=evaluation_config.log_directory,
                console_log_level=logging.INFO,
                file_log_level=logging.INFO,
            )
            # どれを注目物体とするかのparam
            self.__critical_object_filter_config: CriticalObjectFilterConfig = (
                CriticalObjectFilterConfig(
                    evaluator_config=evaluation_config,
                    target_labels=c_cfg["target_labels"],
                    ignore_attributes=c_cfg.get("ignore_attributes", None),
                    max_x_position_list=c_cfg.get("max_x_position_list", None),
                    max_y_position_list=c_cfg.get("max_y_position_list", None),
                    max_distance_list=c_cfg.get("max_distance_list", None),
                    min_distance_list=c_cfg.get("min_distance_list", None),
                    min_point_numbers=c_cfg.get("min_point_numbers", None),
                    confidence_threshold_list=c_cfg.get("confidence_threshold_list", None),
                    target_uuids=c_cfg.get("target_uuids", None),
                )
            )
            # Pass fail を決めるパラメータ
            self.__frame_pass_fail_config: PerceptionPassFailConfig = PerceptionPassFailConfig(
                evaluator_config=evaluation_config,
                target_labels=f_cfg["target_labels"],
                matching_threshold_list=f_cfg.get("matching_threshold_list", None),
                confidence_threshold_list=f_cfg.get("confidence_threshold_list", None),
            )
            self.__evaluator = PerceptionEvaluationManager(evaluation_config=evaluation_config)
            self.__sub_perception = self.create_subscription(
                msg_type,
                "/perception/object_recognition/" + topic_ns + "/objects",
                self.perception_cb,
                1,
            )
            self.__pub_marker_ground_truth = self.create_publisher(
                MarkerArray, "marker/ground_truth", 1
            )
            self.__pub_marker_results = self.create_publisher(MarkerArray, "marker/results", 1)

            self.__current_time = Time().to_msg()
            self.__prev_time = Time().to_msg()

            self.__counter = 0
            self.__timer = self.create_timer(
                1.0,
                self.timer_cb,
                callback_group=self.__timer_group,
                clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            )  # wall timer
            self.__skip_counter = 0
        except KeyError:
            # Immediate termination if the scenario does not contain the required items and is incompatible.
            self.get_logger().error("Scenario format error.")
            rclpy.shutdown()

    def get_frame_id_and_msg_type(self) -> Tuple[str, Union[DetectedObjects, TrackedObjects], str]:
        if self.__evaluation_task in ["detection", "fp_validation"]:
            return "base_link", DetectedObjects, "detection"
        elif self.__evaluation_task == "tracking":
            return "map", TrackedObjects, "tracking"
        else:
            self.get_logger().error(f"Unexpected evaluation task: {self.__evaluation_task}")
            rclpy.shutdown()

    def timer_cb(self):
        self.__current_time = self.get_clock().now().to_msg()
        # self.get_logger().error(f"time: {self.__current_time.sec}.{self.__current_time.nanosec}")
        if self.__current_time.sec > 0:
            if self.__current_time == self.__prev_time:
                self.__counter += 1
            else:
                self.__counter = 0
            self.__prev_time = self.__current_time
            if self.__counter >= 5:
                self.write_log()
                rclpy.shutdown()

    def write_log(self):
        self.__pickle_writer = PickleWriter(self.__pkl_path)
        self.__pickle_writer.dump(self.__evaluator.frame_results)
        if self.__evaluation_task == "fp_validation":
            final_metrics = self.get_fp_result()
            self.__result.add_final_metrics(final_metrics)
            self.__result_writer.write(self.__result)
        else:
            self.get_final_result()
            score_dict = {}
            error_dict = {}
            analyzer = PerceptionAnalyzer3D(self.__evaluator.evaluator_config)
            analyzer.add(self.__evaluator.frame_results)
            score_df, error_df = analyzer.analyze()
            if score_df is not None:
                score_dict = score_df.to_dict()
            if error_df is not None:
                error_dict = (
                    error_df.groupby(level=0).apply(lambda df: df.xs(df.name).to_dict()).to_dict()
                )
            final_metrics = {"Score": score_dict, "Error": error_dict}
            self.__result.add_final_metrics(final_metrics)
            self.__result_writer.write(self.__result)
        self.__result_writer.close()

    def list_dynamic_object_from_ros_msg(
        self, unix_time: int, objects: Union[List[DetectedObject], List[TrackedObject]]
    ) -> List[DynamicObject]:
        estimated_objects: List[DynamicObject] = []
        for perception_object in objects:
            most_probable_classification = get_most_probable_classification(
                perception_object.classification
            )
            label = self.__evaluator.evaluator_config.label_converter.convert_label(
                name=get_label(most_probable_classification)
            )

            uuid = None
            if isinstance(perception_object, TrackedObject):
                uuid = eval_conversions.uuid_from_ros_msg(perception_object.object_id.uuid)

            shape_type = ShapeType.BOUNDING_BOX
            shape_type_num = perception_object.shape.type
            if shape_type_num == 2:
                shape_type = ShapeType.POLYGON

            estimated_object = DynamicObject(
                unix_time=unix_time,
                frame_id=self.__frame_id,
                position=eval_conversions.position_from_ros_msg(
                    perception_object.kinematics.pose_with_covariance.pose.position
                ),
                orientation=eval_conversions.orientation_from_ros_msg(
                    perception_object.kinematics.pose_with_covariance.pose.orientation
                ),
                shape=Shape(
                    shape_type=shape_type,
                    size=eval_conversions.dimensions_from_ros_msg(
                        perception_object.shape.dimensions, shape_type_num
                    ),
                    footprint=eval_conversions.footprint_from_ros_msg(
                        perception_object.shape.footprint
                    ),
                ),
                velocity=eval_conversions.velocity_from_ros_msg(
                    perception_object.kinematics.twist_with_covariance.twist.linear
                ),
                semantic_score=most_probable_classification.probability,
                semantic_label=label,
                uuid=uuid,
            )
            estimated_objects.append(estimated_object)
        return estimated_objects

    def perception_cb(self, msg: Union[DetectedObjects, TrackedObjects]):
        try:
            map_to_baselink = self.__tf_buffer.lookup_transform(
                "map", "base_link", msg.header.stamp, Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            map_to_baselink = TransformStamped()
        # DetectedObjectとTrackedObjectで違う型ではあるが、estimated_objectを作る上で使用している項目は共通で保持しているので同じ関数で処理できる
        unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)
        # 現frameに対応するGround truthを取得
        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(unix_time)
        if ground_truth_now_frame is None:
            self.__skip_counter += 1
        else:
            estimated_objects: List[DynamicObject] = self.list_dynamic_object_from_ros_msg(
                unix_time, msg.objects
            )
            ros_critical_ground_truth_objects = ground_truth_now_frame.objects
            # critical_object_filter_configと、frame_pass_fail_configこの中で動的に変えても良い。
            # 動的に変える条件をかけるようになるまでは、初期化時に一括設定

            frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
                unix_time=unix_time,
                ground_truth_now_frame=ground_truth_now_frame,
                estimated_objects=estimated_objects,
                ros_critical_ground_truth_objects=ros_critical_ground_truth_objects,
                critical_object_filter_config=self.__critical_object_filter_config,
                frame_pass_fail_config=self.__frame_pass_fail_config,
            )
            # write result
            marker_ground_truth, marker_results = self.__result.add_frame(
                frame_result,
                self.__skip_counter,
                msg.header,
                transform_stamped_with_euler_angle(map_to_baselink),
            )
            self.__result_writer.write(self.__result)
            self.__pub_marker_ground_truth.publish(marker_ground_truth)
            self.__pub_marker_results.publish(marker_results)

    def get_final_result(self) -> MetricsScore:
        num_critical_fail: int = sum(
            [
                frame_result.pass_fail_result.get_num_fail()
                for frame_result in self.__evaluator.frame_results
            ]
        )
        logging.info(f"Number of fails for critical objects: {num_critical_fail}")

        # scene metrics score
        final_metric_score = self.__evaluator.get_scene_result()
        logging.info(f"final metrics result {final_metric_score}")
        return final_metric_score

    def get_fp_result(self) -> Dict:
        status_list = get_object_status(self.__evaluator.frame_results)
        gt_status = {}
        for status_info in status_list:
            tp_rate, fp_rate, tn_rate, fn_rate = status_info.get_status_rates()
            # display
            logging.info(
                f"uuid: {status_info.uuid}, "
                # display TP/FP/TN/FN rates per frames
                f"TP: {tp_rate.rate:0.3f}, "
                f"FP: {fp_rate.rate:0.3f}, "
                f"TN: {tn_rate.rate:0.3f}, "
                f"FN: {fn_rate.rate:0.3f}\n"
                # display total or TP/FP/TN/FN frame numbers
                f"Total: {status_info.total_frame_nums}, "
                f"TP: {status_info.tp_frame_nums}, "
                f"FP: {status_info.fp_frame_nums}, "
                f"TN: {status_info.tn_frame_nums}, "
                f"FN: {status_info.fn_frame_nums}",
            )
            gt_status[status_info.uuid] = {
                "rate": {
                    "TP": tp_rate.rate,
                    "FP": fp_rate.rate,
                    "TN": tn_rate.rate,
                    "FN": fn_rate.rate,
                },
                "frame_nums": {
                    "total": status_info.total_frame_nums,
                    "TP": status_info.tp_frame_nums,
                    "FP": status_info.fp_frame_nums,
                    "TN": status_info.tn_frame_nums,
                    "FN": status_info.fn_frame_nums,
                },
            }

        scene_tp_rate, scene_fp_rate, scene_tn_rate, scene_fn_rate = get_scene_rates(status_list)
        logging.info(
            "[scene]"
            f"TP: {scene_tp_rate}, "
            f"FP: {scene_fp_rate}, "
            f"TN: {scene_tn_rate}, "
            f"FN: {scene_fn_rate}"
        )
        return {
            "GroundTruthStatus": gt_status,
            "Scene": {
                "TP": scene_tp_rate,
                "FP": scene_fp_rate,
                "TN": scene_tn_rate,
                "FN": scene_fn_rate,
            },
        }


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    perception_evaluator = PerceptionEvaluator()
    executor.add_node(perception_evaluator)
    executor.spin()
    perception_evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
