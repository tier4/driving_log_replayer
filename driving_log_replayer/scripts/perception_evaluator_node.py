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
from pathlib import Path

from autoware_auto_perception_msgs.msg import DetectedObject
from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import Shape as MsgShape
from autoware_auto_perception_msgs.msg import TrackedObject
from autoware_auto_perception_msgs.msg import TrackedObjects
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
from visualization_msgs.msg import MarkerArray

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.perception import PerceptionResult
from driving_log_replayer.perception import PerceptionScenario
import driving_log_replayer.perception_eval_conversions as eval_conversions


class PerceptionEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, PerceptionScenario, PerceptionResult)
        self._scenario: PerceptionScenario

        self.__p_cfg = self._scenario.Evaluation.PerceptionEvaluationConfig
        self.__c_cfg = self._scenario.Evaluation.CriticalObjectFilterConfig
        self.__f_cfg = self._scenario.Evaluation.PerceptionPassFailConfig
        self.__evaluation_task = self.__p_cfg["evaluation_config_dict"]["evaluation_task"]
        self.__p_cfg["evaluation_config_dict"][
            "label_prefix"
        ] = "autoware"  # Add a fixed value setting
        if not self.check_evaluation_task():
            rclpy.shutdown()

        self.__frame_id: FrameID = FrameID.from_value(self.__frame_id_str)
        evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
            dataset_paths=self._t4_dataset_paths,
            frame_id=self.__frame_id_str,
            result_root_directory=Path(
                self._perception_eval_log_path,
                "result",
                "{TIME}",
            ).as_posix(),
            evaluation_config_dict=self.__p_cfg["evaluation_config_dict"],
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
                target_labels=self.__c_cfg["target_labels"],
                ignore_attributes=self.__c_cfg.get("ignore_attributes", None),
                max_x_position_list=self.__c_cfg.get("max_x_position_list", None),
                max_y_position_list=self.__c_cfg.get("max_y_position_list", None),
                max_distance_list=self.__c_cfg.get("max_distance_list", None),
                min_distance_list=self.__c_cfg.get("min_distance_list", None),
                min_point_numbers=self.__c_cfg.get("min_point_numbers", None),
                confidence_threshold_list=self.__c_cfg.get("confidence_threshold_list", None),
                target_uuids=self.__c_cfg.get("target_uuids", None),
            )
        )
        # Pass fail を決めるパラメータ
        self.__frame_pass_fail_config: PerceptionPassFailConfig = PerceptionPassFailConfig(
            evaluator_config=evaluation_config,
            target_labels=self.__f_cfg["target_labels"],
            matching_threshold_list=self.__f_cfg.get("matching_threshold_list", None),
            confidence_threshold_list=self.__f_cfg.get("confidence_threshold_list", None),
        )
        self.__evaluator = PerceptionEvaluationManager(evaluation_config=evaluation_config)
        self.__sub_perception = self.create_subscription(
            self.__msg_type,
            "/perception/object_recognition/" + self.__topic_ns + "/objects",
            self.perception_cb,
            1,
        )
        self.__pub_marker_ground_truth = self.create_publisher(
            MarkerArray,
            "marker/ground_truth",
            1,
        )
        self.__pub_marker_results = self.create_publisher(MarkerArray, "marker/results", 1)
        self.__skip_counter = 0

    def check_evaluation_task(self) -> bool:
        if self.__evaluation_task in ["detection", "fp_validation"]:
            self.__frame_id_str = "base_link"
            self.__msg_type = DetectedObjects
            self.__topic_ns = "detection"
            return True
        if self.__evaluation_task == "tracking":
            self.__frame_id_str = "map"
            self.__msg_type = TrackedObjects
            self.__topic_ns = "tracking"
            return True
        self.get_logger().error(f"Unexpected evaluation task: {self.__evaluation_task}")
        return False

    def timer_cb(self) -> None:
        super().timer_cb(register_shutdown_func=self.write_metrics)

    def write_metrics(self) -> None:
        self.save_pkl(self.__evaluator.frame_results)
        if self.__evaluation_task == "fp_validation":
            final_metrics = self.get_fp_result()
            self._result.set_final_metrics(final_metrics)
            self._result_writer.write_result(self._result)
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
            self._result.set_final_metrics(final_metrics)
            self._result_writer.write_result(self._result)

    def list_dynamic_object_from_ros_msg(
        self,
        unix_time: int,
        objects: list[DetectedObject] | list[TrackedObject],
    ) -> list[DynamicObject]:
        estimated_objects: list[DynamicObject] = []
        for perception_object in objects:
            most_probable_classification = DLREvaluator.get_most_probable_classification(
                perception_object.classification,
            )
            label = self.__evaluator.evaluator_config.label_converter.convert_label(
                name=DLREvaluator.get_perception_label_str(most_probable_classification),
            )

            uuid = None
            if isinstance(perception_object, TrackedObject):
                uuid = eval_conversions.uuid_from_ros_msg(perception_object.object_id.uuid)

            # check footprint length
            if 1 <= len(perception_object.shape.footprint) < 3:  # noqa
                self.get_logger().error(
                    f"Unexpected footprint length: {len(perception_object.shape.footprint)=}",
                )
                rclpy.shutdown()

            shape_type = ShapeType.BOUNDING_BOX
            shape_type_num = perception_object.shape.type
            if shape_type_num == MsgShape.POLYGON:
                shape_type = ShapeType.POLYGON

            estimated_object = DynamicObject(
                unix_time=unix_time,
                frame_id=self.__frame_id,
                position=eval_conversions.position_from_ros_msg(
                    perception_object.kinematics.pose_with_covariance.pose.position,
                ),
                orientation=eval_conversions.orientation_from_ros_msg(
                    perception_object.kinematics.pose_with_covariance.pose.orientation,
                ),
                shape=Shape(
                    shape_type=shape_type,
                    size=eval_conversions.dimensions_from_ros_msg(
                        perception_object.shape.dimensions,
                        shape_type_num,
                    ),
                    footprint=eval_conversions.footprint_from_ros_msg(
                        perception_object.shape.footprint,
                    ),
                ),
                velocity=eval_conversions.velocity_from_ros_msg(
                    perception_object.kinematics.twist_with_covariance.twist.linear,
                ),
                semantic_score=most_probable_classification.probability,
                semantic_label=label,
                uuid=uuid,
            )
            estimated_objects.append(estimated_object)
        return estimated_objects

    def perception_cb(self, msg: DetectedObjects | TrackedObjects) -> None:
        map_to_baselink = self.lookup_transform(msg.header.stamp)
        # DetectedObjectとTrackedObjectで違う型ではあるが、estimated_objectを作る上で使用している項目は共通で保持しているので同じ関数で処理できる
        unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)
        # 現frameに対応するGround truthを取得
        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(unix_time)
        if ground_truth_now_frame is None:
            self.__skip_counter += 1
            return

        estimated_objects: list[DynamicObject] = self.list_dynamic_object_from_ros_msg(
            unix_time,
            msg.objects,
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
        marker_ground_truth, marker_results = self._result.set_frame(
            frame_result,
            self.__skip_counter,
            msg.header,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
        )
        self._result_writer.write_result(self._result)
        self.__pub_marker_ground_truth.publish(marker_ground_truth)
        self.__pub_marker_results.publish(marker_results)

    def get_final_result(self) -> MetricsScore:
        num_critical_fail: int = sum(
            [
                frame_result.pass_fail_result.get_num_fail()
                for frame_result in self.__evaluator.frame_results
            ],
        )
        logging.info("Number of fails for critical objects: %d", num_critical_fail)

        # scene metrics score
        final_metric_score = self.__evaluator.get_scene_result()
        logging.info("final metrics result %s", final_metric_score)
        return final_metric_score

    def get_fp_result(self) -> dict:
        status_list = get_object_status(self.__evaluator.frame_results)
        gt_status = {}
        for status_info in status_list:
            tp_rate, fp_rate, tn_rate, fn_rate = status_info.get_status_rates()
            # display
            logging.info(
                "uuid: %s, TP: %0.3f, FP: %0.3f, TN: %0.3f, FN: %0.3f\n Total: %s, TP: %s, FP: %s, TN: %s, FN: %s",
                status_info.uuid,
                tp_rate.rate,
                fp_rate.rate,
                tn_rate.rate,
                fn_rate.rate,
                status_info.total_frame_nums,
                status_info.tp_frame_nums,
                status_info.fp_frame_nums,
                status_info.tn_frame_nums,
                status_info.fn_frame_nums,
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
            "[scene] TP: %f, FP: %f, TN: %f, FN: %f",
            scene_tp_rate,
            scene_fp_rate,
            scene_tn_rate,
            scene_fn_rate,
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


@evaluator_main
def main() -> DLREvaluator:
    return PerceptionEvaluator("perception_evaluator")


if __name__ == "__main__":
    main()
