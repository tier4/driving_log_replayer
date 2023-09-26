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
from pathlib import Path
from typing import TYPE_CHECKING

from geometry_msgs.msg import PoseStamped
import numpy as np
from perception_eval.config import SensingEvaluationConfig
from perception_eval.manager import SensingEvaluationManager
from perception_eval.util.logger_config import configure_logger
import rclpy
import ros2_numpy
from rosidl_runtime_py import message_to_ordereddict
from sensor_msgs.msg import PointCloud2
import simplejson as json
from tier4_api_msgs.msg import AwapiAutowareStatus
from visualization_msgs.msg import MarkerArray

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.obstacle_segmentation import get_sensing_frame_config
from driving_log_replayer.obstacle_segmentation import ObstacleSegmentationResult
from driving_log_replayer.obstacle_segmentation_analyzer import default_config_path
from driving_log_replayer.obstacle_segmentation_analyzer import get_graph_data
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer_analyzer.data import convert_str_to_dist_type
from driving_log_replayer_msgs.msg import ObstacleSegmentationInput
from driving_log_replayer_msgs.msg import ObstacleSegmentationMarker
from driving_log_replayer_msgs.msg import ObstacleSegmentationMarkerArray

if TYPE_CHECKING:
    from perception_eval.common.dataset import FrameGroundTruth
    from perception_eval.evaluation.sensing.sensing_frame_result import SensingFrameResult


class ObstacleSegmentationEvaluator(DLREvaluator):
    COUNT_FINISH_PUB_GOAL_POSE = 5

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.check_scenario()
        self.use_t4_dataset()

        self.declare_parameter("vehicle_model", "")
        self.__vehicle_model = (
            self.get_parameter("vehicle_model").get_parameter_value().string_value
        )
        self.__result = ObstacleSegmentationResult(self._condition)
        self.__goal_pose_counter = 0
        self.__goal_pose = DLREvaluator.get_goal_pose_from_t4_dataset(self._t4_dataset_paths[0])

        evaluation_config: SensingEvaluationConfig = SensingEvaluationConfig(
            dataset_paths=self._t4_dataset_paths,
            frame_id="base_link",
            result_root_directory=Path(
                self._perception_eval_log_path,
                "result",
                "{TIME}",
            ).as_posix(),
            evaluation_config_dict=self.__s_cfg["evaluation_config_dict"],
            load_raw_data=False,
        )

        _ = configure_logger(
            log_file_directory=evaluation_config.log_directory,
            console_log_level=logging.INFO,
            file_log_level=logging.INFO,
        )

        self.__evaluator = SensingEvaluationManager(evaluation_config=evaluation_config)

        self.__latest_stop_reasons: list[dict] = []
        self.__sub_awapi_autoware_status = self.create_subscription(
            AwapiAutowareStatus,
            "/awapi/autoware/get/status",
            self.awapi_status_cb,
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
            MarkerArray,
            "marker/non_detection",
            1,
        )
        self.__pub_goal_pose = self.create_publisher(
            PoseStamped,
            "/planning/mission_planning/goal",
            1,
        )
        # for Autoware Evaluator visualization
        self.__pub_graph_topic_rate = self.create_publisher(
            ObstacleSegmentationMarker,
            "graph/topic_rate",
            1,
        )
        self.__pub_graph_detection = self.create_publisher(
            ObstacleSegmentationMarkerArray,
            "graph/detection",
            1,
        )
        self.__pub_graph_non_detection = self.create_publisher(
            ObstacleSegmentationMarker,
            "graph/non_detection",
            1,
        )
        self.__skip_counter = 0

    def check_scenario(self) -> None:
        try:
            self.__s_cfg = self._scenario_yaml_obj["Evaluation"]["SensingEvaluationConfig"]
            self.__s_cfg["evaluation_config_dict"][
                "label_prefix"
            ] = "autoware"  # Add a fixed value setting
        except KeyError:
            self.get_logger().error("Scenario format error.")
            rclpy.shutdown()

    def timer_cb(self) -> None:
        super().timer_cb(
            register_loop_func=self.publish_goal_pose,
            register_shutdown_func=self.write_graph_data,
        )

    def publish_goal_pose(self) -> None:
        if self.__goal_pose_counter < ObstacleSegmentationEvaluator.COUNT_FINISH_PUB_GOAL_POSE:
            self.__goal_pose_counter += 1
            self.__goal_pose.header.stamp = self._current_time
            self.__pub_goal_pose.publish(self.__goal_pose)

    def write_graph_data(self) -> None:
        # debug self.save_pkl(self.__evaluator.frame_results)
        # jsonlを一旦閉じて開きなおす。
        jsonl_file_path = self._result_writer.result_path
        self._result_writer.close()
        detection_dist, pointcloud_numpoints = get_graph_data(
            jsonl_file_path,
            self.__vehicle_model,
            jsonl_file_path.parent,
            default_config_path(),
            convert_str_to_dist_type("euclidean"),
        )
        with jsonl_file_path.open("r+") as jsonl_file:
            last_line = jsonl_file.readlines()[-1]
            try:
                last_line_dict = json.loads(last_line)
                last_line_dict["Frame"] = {
                    "GraphData": {
                        "DetectionDist": detection_dist,
                        "PointcloudCount": pointcloud_numpoints,
                    },
                }
                str_last_line = json.dumps(last_line_dict, ignore_nan=True) + "\n"
                jsonl_file.write(str_last_line)
            except json.JSONDecodeError:
                pass

    def obstacle_segmentation_input_cb(self, msg: ObstacleSegmentationInput) -> None:
        self.__pub_marker_non_detection.publish(msg.marker_array)
        pcd_header = msg.pointcloud.header
        unix_time: int = eval_conversions.unix_time_from_ros_msg(pcd_header)
        ground_truth_now_frame: FrameGroundTruth = self.__evaluator.get_ground_truth_now_frame(
            unix_time=unix_time,
        )
        # Ground truthがない場合はスキップされたことを記録する
        if ground_truth_now_frame is None:
            self.__skip_counter += 1
            return
        # create sensing_frame_config
        frame_ok, sensing_frame_config = get_sensing_frame_config(
            pcd_header,
            self._scenario_yaml_obj,
        )
        if not frame_ok:
            self.__skip_counter += 1
            return
        numpy_pcd = ros2_numpy.numpify(msg.pointcloud)
        pointcloud = np.zeros((numpy_pcd.shape[0], 3))
        pointcloud[:, 0] = numpy_pcd["x"]
        pointcloud[:, 1] = numpy_pcd["y"]
        pointcloud[:, 2] = numpy_pcd["z"]

        non_detection_areas: list[list[tuple[float, float, float]]] = []
        for marker in msg.marker_array.markers:
            non_detection_area: list[tuple[float, float, float]] = []
            for point in marker.points:
                non_detection_area.append((point.x, point.y, point.z))
            non_detection_areas.append(non_detection_area)

        frame_result: SensingFrameResult = self.__evaluator.add_frame_result(
            unix_time=unix_time,
            ground_truth_now_frame=ground_truth_now_frame,
            pointcloud=pointcloud,
            non_detection_areas=non_detection_areas,
            sensing_frame_config=sensing_frame_config,
        )

        # write result
        (
            marker_detection,
            pcd_detection,
            graph_detection,
            pcd_non_detection,
            graph_non_detection,
        ) = self.__result.set_frame(
            frame_result,
            self.__skip_counter,
            DLREvaluator.transform_stamped_with_euler_angle(msg.map_to_baselink),
            self.__latest_stop_reasons,
            pcd_header,
            topic_rate=msg.topic_rate,
        )
        self._result_writer.write_result(self.__result)

        topic_rate_data = ObstacleSegmentationMarker()
        topic_rate_data.header = msg.pointcloud.header
        topic_rate_data.status = (
            ObstacleSegmentationMarker.OK if msg.topic_rate else ObstacleSegmentationMarker.ERROR
        )
        self.__pub_graph_topic_rate.publish(topic_rate_data)

        if marker_detection is not None:
            self.__pub_marker_detection.publish(marker_detection)
        if pcd_detection is not None:
            self.__pub_pcd_detection.publish(pcd_detection)
        if graph_detection is not None:
            self.__pub_graph_detection.publish(graph_detection)
        if pcd_non_detection is not None:
            self.__pub_pcd_non_detection.publish(pcd_non_detection)
        if graph_non_detection is not None:
            self.__pub_graph_non_detection.publish(graph_non_detection)

    def awapi_status_cb(self, msg: AwapiAutowareStatus) -> None:
        self.__latest_stop_reasons = []
        if reasons := msg.stop_reason.stop_reasons:
            for msg_reason in reasons:
                # to debug reason use: self.get_logger().error(f"stop_reason: {msg_reason.reason}")
                if msg_reason.reason == "ObstacleStop":
                    self.__latest_stop_reasons.append(message_to_ordereddict(msg_reason))


@evaluator_main
def main() -> DLREvaluator:
    return ObstacleSegmentationEvaluator("obstacle_segmentation_evaluator")


if __name__ == "__main__":
    main()
