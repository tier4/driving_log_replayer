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
from os.path import expandvars
from pathlib import Path
from typing import TYPE_CHECKING

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from driving_log_replayer_analyzer.data import convert_str_to_dist_type
from driving_log_replayer_msgs.msg import ObstacleSegmentationMarker
from driving_log_replayer_msgs.msg import ObstacleSegmentationMarkerArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import lanelet2  # noqa
from lanelet2_extension_python.utility.query import getLaneletsWithinRange
import numpy as np
from perception_eval.config import SensingEvaluationConfig
from perception_eval.manager import SensingEvaluationManager
from perception_eval.util.logger_config import configure_logger
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import ros2_numpy
from rosidl_runtime_py import message_to_ordereddict
from sensor_msgs.msg import PointCloud2
import simplejson as json
from std_msgs.msg import Header
from tier4_api_msgs.msg import AwapiAutowareStatus
from visualization_msgs.msg import MarkerArray

from log_evaluator.evaluator import DLREvaluator
from log_evaluator.evaluator import evaluator_main
from log_evaluator.lanelet2_util import road_lanelets_from_file
from log_evaluator.lanelet2_util import to_shapely_polygon
from log_evaluator.obstacle_segmentation import default_config_path
from log_evaluator.obstacle_segmentation import get_goal_pose_from_t4_dataset
from log_evaluator.obstacle_segmentation import get_graph_data
from log_evaluator.obstacle_segmentation import get_non_detection_area_in_base_link
from log_evaluator.obstacle_segmentation import get_sensing_frame_config
from log_evaluator.obstacle_segmentation import ObstacleSegmentationResult
from log_evaluator.obstacle_segmentation import ObstacleSegmentationScenario
from log_evaluator.obstacle_segmentation import set_ego_point
from log_evaluator.obstacle_segmentation import transform_proposed_area
import log_evaluator.perception_eval_conversions as eval_conversions

if TYPE_CHECKING:
    from perception_eval.common.dataset import FrameGroundTruth
    from perception_eval.evaluation.sensing.sensing_frame_result import SensingFrameResult

TARGET_DIAG_NAME = "topic_state_monitor_obstacle_segmentation_pointcloud: perception_topic_status"


class ObstacleSegmentationEvaluator(DLREvaluator):
    COUNT_FINISH_PUB_GOAL_POSE = 5

    def __init__(self, name: str) -> None:
        super().__init__(name, ObstacleSegmentationScenario, ObstacleSegmentationResult)
        self._result: ObstacleSegmentationResult

        self._scenario: ObstacleSegmentationScenario
        self.__s_cfg = self._scenario.Evaluation.SensingEvaluationConfig
        self.__s_cfg["evaluation_config_dict"][
            "label_prefix"
        ] = "autoware"  # Add a fixed value setting

        self.declare_parameter("map_path", "")
        map_path = Path(
            expandvars(
                self.get_parameter("map_path").get_parameter_value().string_value,
            ),
            "lanelet2_map.osm",
        ).as_posix()
        self.__road_lanelets = road_lanelets_from_file(map_path)
        if self._scenario.Evaluation.Conditions.NonDetection is not None:
            self.__search_range = (
                self._scenario.Evaluation.Conditions.NonDetection.ProposedArea.search_range()
            )
        else:
            self.__search_range = 0.0
        self.declare_parameter("vehicle_model", "")
        self.__vehicle_model = (
            self.get_parameter("vehicle_model").get_parameter_value().string_value
        )
        self.__goal_pose_counter = 0
        self.__goal_pose = get_goal_pose_from_t4_dataset(self._t4_dataset_paths[0])

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
            PointCloud2,
            "/perception/obstacle_segmentation/pointcloud",
            self.obstacle_segmentation_cb,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self.__sub_diag = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diag_cb,
            100,
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
        self.__topic_rate = False

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

    def obstacle_segmentation_cb(self, msg: PointCloud2) -> None:
        map_to_baselink = self.lookup_transform(msg.header.stamp)
        base_link_to_map = self.lookup_transform(msg.header.stamp, "base_link", "map")
        """
        self.get_logger().error(json.dumps(message_to_ordereddict(map_to_baselink)))
        self.get_logger().error(json.dumps(message_to_ordereddict(base_link_to_map)))
        """
        non_detection_area_markers, non_detection_areas = self.get_non_detection_area(
            msg.header,
            map_to_baselink,
            base_link_to_map,
        )
        self.__pub_marker_non_detection.publish(non_detection_area_markers)
        pcd_header = msg.header
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
            self._scenario,
        )
        if not frame_ok:
            self.__skip_counter += 1
            return
        numpy_pcd = ros2_numpy.numpify(msg)
        pointcloud = np.zeros((numpy_pcd.shape[0], 3))
        pointcloud[:, 0] = numpy_pcd["x"]
        pointcloud[:, 1] = numpy_pcd["y"]
        pointcloud[:, 2] = numpy_pcd["z"]

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
        ) = self._result.set_frame(
            frame_result,
            self.__skip_counter,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_stop_reasons,
            pcd_header,
            topic_rate=self.__topic_rate,
        )
        self._result_writer.write_result(self._result)

        topic_rate_data = ObstacleSegmentationMarker()
        topic_rate_data.header = msg.header
        topic_rate_data.status = (
            ObstacleSegmentationMarker.OK if self.__topic_rate else ObstacleSegmentationMarker.ERROR
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

    def diag_cb(self, msg: DiagnosticArray) -> None:
        diag_status: DiagnosticStatus = msg.status[0]
        if diag_status.name == TARGET_DIAG_NAME:
            return
        if diag_status.level >= DiagnosticStatus.ERROR:
            self.__topic_rate = False
        else:
            self.__topic_rate = True

    def get_non_detection_area(
        self,
        header: Header,
        map_to_baselink: TransformStamped,
        base_link_to_map: TransformStamped,
    ) -> tuple[MarkerArray, list]:
        non_detection_area_markers = MarkerArray()
        non_detection_areas = []
        cond_non_detection = self._scenario.Evaluation.Conditions.NonDetection
        if cond_non_detection is None:
            return non_detection_area_markers, non_detection_areas
        s_proposed_area = cond_non_detection.ProposedArea
        # get proposed_area in map
        proposed_area_in_map, average_z = transform_proposed_area(
            s_proposed_area,
            header,
            map_to_baselink,
        )
        # get intersection
        marker_id = 0
        ego_point = set_ego_point(map_to_baselink)
        near_road_lanelets = getLaneletsWithinRange(
            self.__road_lanelets,
            ego_point,
            self.__search_range,
        )
        for road in near_road_lanelets:
            poly_lanelet = to_shapely_polygon(road)
            i_area = poly_lanelet.intersection(proposed_area_in_map)
            if not i_area.is_empty:
                marker_id += 1
                marker, area = get_non_detection_area_in_base_link(
                    i_area,
                    header,
                    s_proposed_area.z_min,
                    s_proposed_area.z_max,
                    average_z,
                    base_link_to_map,
                    marker_id,
                )
                non_detection_area_markers.markers.append(marker)  # base_link
                non_detection_areas.append(area)  # base_link
        # create marker and polygon for perception_eval
        return non_detection_area_markers, non_detection_areas


@evaluator_main
def main() -> DLREvaluator:
    return ObstacleSegmentationEvaluator("obstacle_segmentation_evaluator")


if __name__ == "__main__":
    main()
