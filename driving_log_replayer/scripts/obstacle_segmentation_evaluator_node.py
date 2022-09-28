#!/usr/bin/env python3

# Copyright (c) 2022 TierIV.inc
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
from typing import Dict
from typing import List
from typing import Tuple

from driving_log_replayer.node_common import get_goal_pose_from_t4_dataset
from driving_log_replayer.node_common import transform_stamped_with_euler_angle
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter
from driving_log_replayer_msgs.msg import ObstacleSegmentationInput
from geometry_msgs.msg import PoseStamped
import numpy as np
from perception_eval.common.dataset import FrameGroundTruth
from perception_eval.common.object import DynamicObject
from perception_eval.config.sensing_evaluation_config import SensingEvaluationConfig
from perception_eval.evaluation.sensing.sensing_frame_result import SensingFrameResult
from perception_eval.evaluation.sensing.sensing_result import DynamicObjectWithSensingResult
from perception_eval.manager.sensing_evaluation_manager import SensingEvaluationManager
from perception_eval.util.logger_config import configure_logger
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
import ros2_numpy
from rosidl_runtime_py import message_to_ordereddict
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from tier4_planning_msgs.msg import StopReasonArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import yaml


def get_box_marker(
    ground_truth_obj: DynamicObject,
    header: Header,
    namespace: str,
    marker_id: int,
    color: ColorRGBA,
) -> Tuple[Marker, Marker, Dict]:
    gt_state = ground_truth_obj.state
    bbox, uuid = eval_conversions.object_state_to_ros_box_and_uuid(
        gt_state, header, namespace, marker_id, color, ground_truth_obj.uuid
    )
    info_dict = {
        "Annotation": {
            "Scale": message_to_ordereddict(bbox.scale),
            "Position": message_to_ordereddict(bbox.pose),
            "UUID": ground_truth_obj.uuid,
        }
    }
    return bbox, uuid, info_dict


def summarize_frame_container(
    container: List[DynamicObjectWithSensingResult],
    header: Header,
    color: ColorRGBA,
    summary_dict: Dict,
    counter: int,
    marker_array: MarkerArray,
    pcd: np.ndarray,
) -> Tuple[Dict, int, MarkerArray, np.ndarray]:
    for result in container:
        pcd_result = np.zeros(
            result.inside_pointcloud_num,
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        pcd_result["x"] = result.inside_pointcloud[:, 0]
        pcd_result["y"] = result.inside_pointcloud[:, 1]
        pcd_result["z"] = result.inside_pointcloud[:, 2]
        pcd = np.concatenate([pcd, pcd_result])

        bbox, uuid, info_dict = get_box_marker(
            result.ground_truth_object,
            header,
            "detection",
            counter := counter + 1,  # アノテーションは基本的にどのフレームでもあるので、uuidで指定した分だけ全体でカウンターが回る。
            color,
        )
        marker_array.markers.append(bbox)
        marker_array.markers.append(uuid)
        info_dict["PointCloud"] = {
            "NumPoints": result.inside_pointcloud_num,
            "Nearest": result.nearest_point.tolist() if result.nearest_point is not None else [],
        }
        summary_dict["Info"].append(info_dict)
    return summary_dict, counter, marker_array, pcd


def summarize_detection(
    frame_result: SensingFrameResult, topic_rate: bool, header: Header
) -> Tuple[Dict, str, MarkerArray, PointCloud2]:
    success = "Fail"
    if len(frame_result.detection_fail_results) == 0 and topic_rate:
        success = "Success"
    if len(frame_result.detection_warning_results) != 0:
        success = "Warn"
    summary_dict = {"Result": success, "Info": []}

    # initialize
    marker_array = MarkerArray()
    # create detection pcd
    pcd = np.zeros(
        0,
        dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
    )
    counter = 0

    color_success = ColorRGBA()
    color_success.r = 0.0
    color_success.g = 1.0
    color_success.b = 0.0
    color_success.a = 0.3

    summary_dict, counter, marker_array, pcd = summarize_frame_container(
        frame_result.detection_success_results,
        header,
        color_success,
        summary_dict,
        counter,
        marker_array,
        pcd,
    )

    color_fail = ColorRGBA()
    color_fail.r = 1.0
    color_fail.g = 0.0
    color_fail.b = 0.0
    color_fail.a = 0.3

    summary_dict, counter, marker_array, pcd = summarize_frame_container(
        frame_result.detection_fail_results,
        header,
        color_fail,
        summary_dict,
        counter,
        marker_array,
        pcd,
    )

    color_warn = ColorRGBA()
    color_warn.r = 1.0
    color_warn.g = 0.65
    color_warn.b = 0.0
    color_warn.a = 0.3

    summary_dict, counter, marker_array, pcd = summarize_frame_container(
        frame_result.detection_warning_results,
        header,
        color_warn,
        summary_dict,
        counter,
        marker_array,
        pcd,
    )

    pcd_detection = ros2_numpy.msgify(PointCloud2, pcd)
    pcd_detection.header = header

    return summary_dict, success, marker_array, pcd_detection


def summarize_numpy_pointcloud(
    list_pcd: List[np.ndarray], header: Header
) -> Tuple[PointCloud2, np.ndarray]:
    numpy_pcd = np.zeros(
        0,
        dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
    )
    dist_array = np.array([])
    for pcd in list_pcd:
        dist = np.linalg.norm(pcd, ord=2, axis=1)
        dist_array = np.concatenate([dist_array, dist])
        num_points = pcd.shape[0]
        pcd_data = np.zeros(
            num_points,
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        pcd_data["x"] = pcd[:, 0]
        pcd_data["y"] = pcd[:, 1]
        pcd_data["z"] = pcd[:, 2]
        numpy_pcd = np.concatenate([numpy_pcd, pcd_data])
    ros_pcd = ros2_numpy.msgify(PointCloud2, numpy_pcd)
    ros_pcd.header = header
    return ros_pcd, dist_array


def summarize_non_detection(
    pcd_list: List[np.ndarray], topic_rate: bool, header: Header
) -> Tuple[Dict, str, PointCloud2]:
    success = "Success" if len(pcd_list) == 0 and topic_rate else "Fail"
    summary_dict = {"Result": success}
    ros_pcd, dist_array = summarize_numpy_pointcloud(pcd_list, header)

    # 詳細情報の追記を書く
    dist_dict = {}
    # print(dist_array)
    for i in range(100):
        dist_dict[f"{i}-{i+1}"] = np.count_nonzero((i <= dist_array) & (dist_array < i + 1))
    # NonDetectionは結果はInfoに入るのは１個しかないがDetectionに合わせてlistにしておく
    summary_dict["Info"] = [
        {
            "PointCloud": {"NumPoints": dist_array.shape[0], "Distance": dist_dict},
        }
    ]
    return summary_dict, success, ros_pcd


class ObstacleSegmentationResult(ResultBase):
    def __init__(self, condition: Dict):
        super().__init__()
        self.__detection_pass_rate = condition["ObstacleDetection"]["PassRate"]
        self.__non_detection_pass_rate = condition["NonDetection"]["PassRate"]
        self.__detection_total = 0
        self.__detection_success = 0
        self.__detection_warn = 0
        self.__non_detection_total = 0
        self.__non_detection_success = 0

    def update(self):
        detection_rate = (
            0.0
            if self.__detection_total == 0
            else self.__detection_success / self.__detection_total * 100.0
        )
        non_detection_rate = (
            0.0
            if self.__non_detection_total == 0
            else self.__non_detection_success / self.__non_detection_total * 100.0
        )
        success_detection = detection_rate >= self.__detection_pass_rate
        success_non_detection = non_detection_rate >= self.__non_detection_pass_rate

        summary_str = (
            f"detection: {self.__detection_success} / {self.__detection_total } -> {detection_rate:.2f}% "
            + f"detection_warn: {self.__detection_warn} "
            + f"non_detection: {self.__non_detection_success} / {self.__non_detection_total} -> {non_detection_rate:.2f}%"
        )

        if success_detection and success_non_detection:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        elif success_detection and not success_non_detection:
            self._success = False
            self._summary = f"NonDetection Failed: {summary_str}"
        elif not success_detection and success_non_detection:
            self._success = False
            self._summary = f"Detection Failed: {summary_str}"
        elif not success_detection and not success_non_detection:
            self._success = False
            self._summary = f"Detection and NonDetection Failed: {summary_str}"

    def add_frame(
        self,
        frame: SensingFrameResult,
        skip: int,
        map_to_baselink: Dict,
        stop_reasons: List[Dict],
        topic_rate: bool,
        header: Header,
    ) -> Tuple[MarkerArray, PointCloud2, PointCloud2]:
        self.__non_detection_total += 1
        out_frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }

        (
            out_frame["Detection"],
            detection_success,
            marker_detection,
            pcd_detection,
        ) = summarize_detection(frame, topic_rate, header)
        (
            out_frame["NonDetection"],
            non_detection_success,
            pcd_non_detection,
        ) = summarize_non_detection(frame.pointcloud_failed_non_detection, topic_rate, header)
        out_frame["StopReasons"] = stop_reasons
        out_frame["TopicRate"] = str(topic_rate)

        if detection_success == "Warn":
            self.__detection_warn += 1
        else:
            self.__detection_total += 1
            if detection_success == "Success":
                self.__detection_success += 1
        if non_detection_success == "Success":
            self.__non_detection_success += 1

        # update current frame
        self._frame = out_frame
        # update result
        self.update()
        return marker_detection, pcd_detection, pcd_non_detection


class ObstacleSegmentationEvaluator(Node):
    def __init__(self):
        super().__init__("obstacle_segmentation_evaluator")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("result_json_path", "")
        self.declare_parameter("t4_dataset_path", "")
        self.declare_parameter("result_archive_path", "")

        self.__timer_group = MutuallyExclusiveCallbackGroup()

        scenario_path = os.path.expandvars(
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )
        self.__scenario_yaml_obj = None
        with open(scenario_path, "r") as scenario_file:
            self.__scenario_yaml_obj = yaml.safe_load(scenario_file)
        self.__result_json_path = os.path.expandvars(
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )
        self.__t4_dataset_paths = [
            os.path.expandvars(
                self.get_parameter("t4_dataset_path").get_parameter_value().string_value
            )
        ]
        self.__perception_eval_log_path = os.path.join(
            os.path.dirname(self.__result_json_path), "perception_eval_log"
        )
        # read setting from scenario yaml
        self.__condition = self.__scenario_yaml_obj["Evaluation"]["Conditions"]
        self.__result = ObstacleSegmentationResult(self.__condition)
        self.__goal_pose_counter = 0
        self.__goal_pose = get_goal_pose_from_t4_dataset(self.__t4_dataset_paths[0])
        self.__result_writer = ResultWriter(
            self.__result_json_path, self.get_clock(), self.__condition
        )

        s_cfg = self.__scenario_yaml_obj["Evaluation"]["SensingEvaluationConfig"]

        evaluation_config: SensingEvaluationConfig = SensingEvaluationConfig(
            dataset_paths=self.__t4_dataset_paths,
            frame_id="base_link",
            merge_similar_labels=False,
            does_use_pointcloud=False,
            result_root_directory=os.path.join(self.__perception_eval_log_path, "result", "{TIME}"),
            evaluation_config_dict=s_cfg["evaluation_config_dict"],
        )

        _ = configure_logger(
            log_file_directory=evaluation_config.log_directory,
            console_log_level=logging.INFO,
            file_log_level=logging.INFO,
        )

        self.__evaluator = SensingEvaluationManager(evaluation_config=evaluation_config)

        self.__latest_stop_reasons: List[Dict] = []
        self.__sub_stop_reasons = self.create_subscription(
            StopReasonArray,
            "/planning/scenario_planning/status/stop_reasons",
            self.stop_reason_cb,
            1,
        )

        self.__sub_obstacle_segmentation = self.create_subscription(
            ObstacleSegmentationInput,
            "obstacle_segmentation/input",
            self.obstacle_segmentation_input_cb,
            10,
        )
        self.__pub_pcd_detection = self.create_publisher(PointCloud2, "pcd/detection", 1)
        self.__pub_pcd_non_detection = self.create_publisher(PointCloud2, "pcd/non_detection", 1)
        self.__pub_marker_detection = self.create_publisher(MarkerArray, "marker/detection", 1)
        self.__pub_marker_non_detection = self.create_publisher(
            MarkerArray, "marker/non_detection", 1
        )
        self.__pub_goal_pose = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", 1
        )

        self.__current_time = Time().to_msg()
        self.__prev_time = Time().to_msg()

        self.__shutdown_counter = 0
        self.__timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self.__timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer
        self.__skip_counter = 0

    def timer_cb(self):
        self.__current_time = self.get_clock().now().to_msg()
        # self.get_logger().error(f"time: {self.__current_time.sec}.{self.__current_time.nanosec}")
        if self.__current_time.sec > 0:
            self.__goal_pose_counter = self.__goal_pose_counter + 1
            if self.__goal_pose_counter <= 5:
                self.__goal_pose.header.stamp = self.__current_time
                self.__pub_goal_pose.publish(self.__goal_pose)
            if self.__current_time == self.__prev_time:
                self.__shutdown_counter += 1
            else:
                self.__shutdown_counter = 0
            self.__prev_time = self.__current_time
            if self.__shutdown_counter >= 5:
                self.get_final_result()
                # for debug
                # from driving_log_replayer.result import PickleWriter

                # self.__pkl_path = os.path.join(
                #     os.path.dirname(self.__result_json_path), "frame.pkl"
                # )
                # self.__pickle_writer = PickleWriter(self.__pkl_path)
                # self.__pickle_writer.dump(self.__evaluator.frame_results)
                self.__result_writer.close()
                rclpy.shutdown()

    def obstacle_segmentation_input_cb(self, msg: ObstacleSegmentationInput):
        self.__pub_marker_non_detection.publish(msg.marker_array)
        pcd_header = msg.pointcloud.header
        unix_time: int = eval_conversions.unix_time_from_ros_msg(pcd_header)
        ground_truth_now_frame: FrameGroundTruth = self.__evaluator.get_ground_truth_now_frame(
            unix_time=unix_time,
        )
        # Ground truthがない場合はスキップされたことを記録する
        if ground_truth_now_frame is None:
            self.__skip_counter += 1
        else:
            numpy_pcd = ros2_numpy.numpify(msg.pointcloud)
            pointcloud = np.zeros((numpy_pcd.shape[0], 3))
            pointcloud[:, 0] = numpy_pcd["x"]
            pointcloud[:, 1] = numpy_pcd["y"]
            pointcloud[:, 2] = numpy_pcd["z"]

            non_detection_areas: List[List[Tuple[float, float, float]]] = []
            for marker in msg.marker_array.markers:
                non_detection_area: List[Tuple[float, float, float]] = []
                for point in marker.points:
                    non_detection_area.append((point.x, point.y, point.z))
                non_detection_areas.append(non_detection_area)

            frame_result: SensingFrameResult = self.__evaluator.add_frame_result(
                unix_time=unix_time,
                ground_truth_now_frame=ground_truth_now_frame,
                pointcloud=pointcloud,
                non_detection_areas=non_detection_areas,
            )

            # write result
            marker_detection, pcd_detection, pcd_non_detection = self.__result.add_frame(
                frame_result,
                self.__skip_counter,
                transform_stamped_with_euler_angle(msg.map_to_baselink),
                self.__latest_stop_reasons,
                msg.topic_rate.data,
                pcd_header,
            )
            self.__result_writer.write(self.__result)

            self.__pub_pcd_detection.publish(pcd_detection)
            self.__pub_pcd_non_detection.publish(pcd_non_detection)
            self.__pub_marker_detection.publish(marker_detection)
            # self.visualize(frame_result)

    def stop_reason_cb(self, msg: StopReasonArray):
        reasons = []
        for msg_reason in msg.stop_reasons:
            if msg_reason.reason == "ObstacleStop":
                reasons.append(message_to_ordereddict(msg_reason))
        self.__latest_stop_reasons = reasons

    def get_final_result(self) -> None:
        """Output the evaluation results on the command line."""
        # use case fail object num
        num_use_case_fail: int = 0
        for frame_results in self.__evaluator.frame_results:
            num_use_case_fail += len(frame_results.detection_fail_results)
        logging.warning(f"{num_use_case_fail} fail results.")

    @staticmethod
    def visualize(frame_result: SensingFrameResult) -> None:
        """Visualize results per frame.

        Args:
            frame_result (SensingFrameResult)
        """
        if len(frame_result.detection_fail_results) > 0:
            logging.warning(f"Fail {len(frame_result.detection_fail_results)} detection.")
            for fail_result in frame_result.detection_fail_results:
                logging.info(
                    f"[FAIL] Inside points: {fail_result.inside_pointcloud_num}, Is detected: {fail_result.is_detected}"
                )
        else:
            logging.info("all detections were succeeded.")

        for success_result in frame_result.detection_success_results:
            logging.info(
                f"[SUCCESS] Inside points: {success_result.inside_pointcloud_num}, "
                f"Is detected: {success_result.is_detected}, "
                f"Nearest point: {success_result.nearest_point}\n"
            )

        if len(frame_result.pointcloud_failed_non_detection) > 0:
            logging.warning(
                f"The number of Failed non-detection pointcloud: {len(frame_result.pointcloud_failed_non_detection)}"
            )


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    obstacle_segmentation_evaluator = ObstacleSegmentationEvaluator()
    executor.add_node(obstacle_segmentation_evaluator)
    executor.spin()
    obstacle_segmentation_evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
