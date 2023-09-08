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
from pathlib import Path
import sys
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple
from typing import TYPE_CHECKING

from geometry_msgs.msg import PoseStamped
import numpy as np
from perception_eval.common.object import DynamicObject
from perception_eval.config import SensingEvaluationConfig
from perception_eval.evaluation.sensing.sensing_frame_config import SensingFrameConfig
from perception_eval.evaluation.sensing.sensing_frame_result import SensingFrameResult
from perception_eval.evaluation.sensing.sensing_result import DynamicObjectWithSensingResult
from perception_eval.manager import SensingEvaluationManager
from perception_eval.util.logger_config import configure_logger
import rclpy
import ros2_numpy
from rosidl_runtime_py import message_to_ordereddict
from sensor_msgs.msg import PointCloud2
import simplejson as json
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from tier4_api_msgs.msg import AwapiAutowareStatus
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.obstacle_segmentation_analyzer import default_config_path
from driving_log_replayer.obstacle_segmentation_analyzer import get_graph_data
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.result import ResultBase
from driving_log_replayer_analyzer.data import convert_str_to_dist_type
from driving_log_replayer_msgs.msg import ObstacleSegmentationInput
from driving_log_replayer_msgs.msg import ObstacleSegmentationMarker
from driving_log_replayer_msgs.msg import ObstacleSegmentationMarkerArray

if TYPE_CHECKING:
    from perception_eval.common.dataset import FrameGroundTruth


def get_box_marker(
    ground_truth_obj: DynamicObject,
    header: Header,
    namespace: str,
    marker_id: int,
    color: ColorRGBA,
) -> Tuple[Marker, Marker, Dict]:
    gt_state = ground_truth_obj.state
    bbox, uuid = eval_conversions.object_state_to_ros_box_and_uuid(
        gt_state,
        header,
        namespace,
        marker_id,
        color,
        ground_truth_obj.uuid,
    )
    info_dict = {
        "Annotation": {
            "Scale": message_to_ordereddict(bbox.scale),
            "Position": message_to_ordereddict(bbox.pose),
            "UUID": ground_truth_obj.uuid,
            "StampFloat": ground_truth_obj.unix_time / pow(10, 6),
        },
    }
    return bbox, uuid, info_dict


def summarize_frame_container(
    container: List[DynamicObjectWithSensingResult],
    header: Header,
    color: ColorRGBA,
    info: List,
    counter: int,
    marker_array: MarkerArray,
    pcd: np.ndarray,
    graph_detection: ObstacleSegmentationMarkerArray,
    result_status: int,
) -> Tuple[Dict, int, MarkerArray, np.ndarray, ObstacleSegmentationMarkerArray]:
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
            "Stamp": message_to_ordereddict(header.stamp),
        }
        info.append(info_dict)
        marker = ObstacleSegmentationMarker(
            header=header,
            number_of_pointcloud=result.inside_pointcloud_num,
            status=result_status,
            bbox_uuid=result.ground_truth_object.uuid,
        )
        graph_detection.markers.append(marker)
    return info, counter, marker_array, pcd, graph_detection


def summarize_numpy_pointcloud(
    list_pcd: List[np.ndarray],
    header: Header,
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


def get_sensing_frame_config(
    pcd_header: Header,
    scenario_yaml_obj: Dict,
) -> Tuple[bool, Optional[SensingFrameConfig]]:
    detection_config = scenario_yaml_obj["Evaluation"]["Conditions"].get("Detection", None)
    if detection_config is None:
        return True, None
    bbox_conf = detection_config.get("BoundingBoxConfig", None)
    if bbox_conf is None:
        return True, None
    target_uuids = []
    for uuid_dict in bbox_conf:
        for uuid, v in uuid_dict.items():
            # uuid: str, v: Dict
            start: Optional[float] = v.get("Start", None)
            end: Optional[float] = v.get("End", None)
            if start is None:
                start = 0.0
            if end is None:
                end = sys.float_info.max
            stamp_float = pcd_header.stamp.sec + (pcd_header.stamp.nanosec / pow(10, 9))
            if start <= stamp_float <= end:
                target_uuids.append(uuid)
    if target_uuids == []:
        return False, None
    e_conf = scenario_yaml_obj["Evaluation"]["SensingEvaluationConfig"]["evaluation_config_dict"]
    sensing_frame_config = SensingFrameConfig(
        target_uuids=target_uuids,
        box_scale_0m=e_conf["box_scale_0m"],
        box_scale_100m=e_conf["box_scale_100m"],
        min_points_threshold=e_conf["min_points_threshold"],
    )
    return True, sensing_frame_config


class ObstacleSegmentationResult(ResultBase):
    def __init__(self, condition: Dict) -> None:
        super().__init__()
        self.__condition_detection: Optional[Dict] = condition.get("Detection", None)
        self.__condition_non_detection: Optional[Dict] = condition.get("NonDetection", None)

        self.__detection_total = 0
        self.__detection_success = 0
        self.__detection_msg = "NotTested"
        self.__detection_result = True
        self.__detection_warn = 0
        self.__non_detection_total = 0
        self.__non_detection_success = 0
        self.__non_detection_msg = "NotTested"
        self.__non_detection_result = True

    def update(self) -> None:
        summary_str = f"Detection: {self.__detection_msg} NonDetection: {self.__non_detection_msg}"
        if self.__detection_result and self.__non_detection_result:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        elif self.__detection_result and not self.__non_detection_result:
            self._success = False
            self._summary = f"NonDetection Failed: {summary_str}"
        elif not self.__detection_result and self.__non_detection_result:
            self._success = False
            self._summary = f"Detection Failed: {summary_str}"
        elif not self.__detection_result and not self.__non_detection_result:
            self._success = False
            self._summary = f"Detection and NonDetection Failed: {summary_str}"

    def summarize_detection(
        self,
        frame_result: SensingFrameResult,
        topic_rate: bool,
        header: Header,
    ) -> Tuple[
        Dict,
        Optional[MarkerArray],
        Optional[PointCloud2],
        Optional[ObstacleSegmentationMarkerArray],
    ]:
        result = "Skipped"
        info = []
        marker_array = None
        pcd_detection = None
        graph_detection = None

        if self.__condition_detection is not None:
            result = "Fail"
            if len(frame_result.detection_warning_results) != 0:
                result = "Warn"
                self.__detection_warn += 1
            else:
                self.__detection_total += 1
                # Failure if there is no point cloud and no success
                if (
                    len(frame_result.detection_fail_results) == 0
                    and len(frame_result.detection_success_results) != 0
                    and topic_rate
                ):
                    result = "Success"
                    self.__detection_success += 1

            detection_rate = (
                0.0
                if self.__detection_total == 0
                else self.__detection_success / self.__detection_total * 100.0
            )
            self.__detection_result = detection_rate >= self.__condition_detection["PassRate"]
            self.__detection_msg = f"Detection: {self.__detection_success} / {self.__detection_total } -> {detection_rate:.2f}% (Warn: {self.__detection_warn})"

            # initialize
            marker_array = MarkerArray()
            # create detection pcd
            pcd = np.zeros(
                0,
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
            )
            counter = 0
            graph_detection = ObstacleSegmentationMarkerArray()

            color_success = (
                ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)
                if result == "Success"
                else ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
            )
            info, counter, marker_array, pcd, graph_detection = summarize_frame_container(
                frame_result.detection_success_results,
                header,
                color_success,
                info,
                counter,
                marker_array,
                pcd,
                graph_detection,
                ObstacleSegmentationMarker.OK
                if result == "Success"
                else ObstacleSegmentationMarker.ERROR,
            )

            color_fail = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
            info, counter, marker_array, pcd, graph_detection = summarize_frame_container(
                frame_result.detection_fail_results,
                header,
                color_fail,
                info,
                counter,
                marker_array,
                pcd,
                graph_detection,
                ObstacleSegmentationMarker.ERROR,
            )

            color_warn = ColorRGBA(r=1.0, g=0.65, b=0.0, a=0.3)
            info, counter, marker_array, pcd, graph_detection = summarize_frame_container(
                frame_result.detection_warning_results,
                header,
                color_warn,
                info,
                counter,
                marker_array,
                pcd,
                graph_detection,
                ObstacleSegmentationMarker.WARN,
            )

            pcd_detection = ros2_numpy.msgify(PointCloud2, pcd)
            pcd_detection.header = header
        return {"Result": result, "Info": info}, marker_array, pcd_detection, graph_detection

    def summarize_non_detection(
        self,
        pcd_list: List[np.ndarray],
        topic_rate: bool,
        header: Header,
    ) -> Tuple[Dict, Optional[PointCloud2], Optional[ObstacleSegmentationMarker]]:
        result = "Skipped"
        info = []
        ros_pcd = None
        graph_non_detection = None

        if self.__condition_non_detection is not None:
            result = "Fail"
            self.__non_detection_total += 1
            if len(pcd_list) == 0 and topic_rate:
                result = "Success"
                self.__non_detection_success += 1
            non_detection_rate = self.__non_detection_success / self.__non_detection_total * 100.0
            self.__non_detection_result = (
                non_detection_rate >= self.__condition_non_detection["PassRate"]
            )
            self.__non_detection_msg = f"NonDetection: {self.__non_detection_success} / {self.__non_detection_total} -> {non_detection_rate:.2f}%"

            ros_pcd, dist_array = summarize_numpy_pointcloud(pcd_list, header)
            graph_non_detection = ObstacleSegmentationMarker()
            graph_non_detection.header = header
            graph_non_detection.status = (
                ObstacleSegmentationMarker.ERROR
                if result == "Fail"
                else ObstacleSegmentationMarker.OK
            )
            graph_non_detection.number_of_pointcloud = dist_array.shape[0]

            if result == "Fail":
                # 詳細情報の追記を書く
                dist_dict = {}
                for i in range(100):
                    dist_dict[f"{i}-{i+1}"] = np.count_nonzero(
                        (i <= dist_array) & (dist_array < i + 1),
                    )
                # NonDetectionは結果はInfoに入るのは1個しかないがDetectionに合わせてlistにしておく
                info.append(
                    {
                        "PointCloud": {"NumPoints": dist_array.shape[0], "Distance": dist_dict},
                    },
                )
        return {"Result": result, "Info": info}, ros_pcd, graph_non_detection

    def set_frame(
        self,
        frame: SensingFrameResult,
        skip: int,
        map_to_baselink: Dict,
        stop_reasons: List[Dict],
        topic_rate: bool,
        header: Header,
    ) -> Tuple[
        Optional[MarkerArray],
        Optional[PointCloud2],
        Optional[ObstacleSegmentationMarkerArray],
        Optional[PointCloud2],
        Optional[ObstacleSegmentationMarker],
    ]:
        out_frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        (
            out_frame["Detection"],
            marker_detection,
            pcd_detection,
            graph_detection,
        ) = self.summarize_detection(frame, topic_rate, header)
        (
            out_frame["NonDetection"],
            pcd_non_detection,
            graph_non_detection,
        ) = self.summarize_non_detection(frame.pointcloud_failed_non_detection, topic_rate, header)
        out_frame["StopReasons"] = stop_reasons
        out_frame["TopicRate"] = str(topic_rate)
        # update current frame
        self._frame = out_frame
        # update result
        self.update()
        return (
            marker_detection,
            pcd_detection,
            graph_detection,
            pcd_non_detection,
            graph_non_detection,
        )


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
            result_root_directory=os.path.join(self._perception_eval_log_path, "result", "{TIME}"),
            evaluation_config_dict=self.__s_cfg["evaluation_config_dict"],
            load_raw_data=False,
        )

        _ = configure_logger(
            log_file_directory=evaluation_config.log_directory,
            console_log_level=logging.INFO,
            file_log_level=logging.INFO,
        )

        self.__evaluator = SensingEvaluationManager(evaluation_config=evaluation_config)

        self.__latest_stop_reasons: List[Dict] = []
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
        # jsonlを一旦閉じて開きなおす
        self._result_writer.close()
        jsonl_file_path = Path(
            os.path.splitext(os.path.expandvars(self._result_json_path))[0] + ".jsonl",
        )
        detection_dist, pointcloud_numpoints = get_graph_data(
            jsonl_file_path,
            self.__vehicle_model,
            jsonl_file_path.parent,
            default_config_path(),
            convert_str_to_dist_type("euclidean"),
        )
        with open(jsonl_file_path, "r+") as jsonl_file:
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
            msg.topic_rate,
            pcd_header,
        )
        self._result_writer.write(self.__result)

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
