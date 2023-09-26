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

import sys

import numpy as np
from perception_eval.common.object import DynamicObject
from perception_eval.evaluation.sensing.sensing_frame_config import SensingFrameConfig
from perception_eval.evaluation.sensing.sensing_frame_result import SensingFrameResult
from perception_eval.evaluation.sensing.sensing_result import DynamicObjectWithSensingResult
import ros2_numpy
from rosidl_runtime_py import message_to_ordereddict
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.result import ResultBase
from driving_log_replayer_msgs.msg import ObstacleSegmentationMarker
from driving_log_replayer_msgs.msg import ObstacleSegmentationMarkerArray


def get_box_marker(
    ground_truth_obj: DynamicObject,
    header: Header,
    namespace: str,
    marker_id: int,
    color: ColorRGBA,
) -> tuple[Marker, Marker, dict]:
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
    container: list[DynamicObjectWithSensingResult],
    header: Header,
    color: ColorRGBA,
    info: list,
    counter: int,
    marker_array: MarkerArray,
    pcd: np.ndarray,
    graph_detection: ObstacleSegmentationMarkerArray,
    result_status: int,
) -> tuple[dict, int, MarkerArray, np.ndarray, ObstacleSegmentationMarkerArray]:
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
    list_pcd: list[np.ndarray],
    header: Header,
) -> tuple[PointCloud2, np.ndarray]:
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
    scenario_yaml_obj: dict,
) -> tuple[bool, SensingFrameConfig | None]:
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
            start: float | None = v.get("Start", None)
            end: float | None = v.get("End", None)
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
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__condition_detection: dict | None = condition.get("Detection", None)
        self.__condition_non_detection: dict | None = condition.get("NonDetection", None)

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
        header: Header,
        *,
        topic_rate: bool,
    ) -> tuple[
        dict,
        MarkerArray | None,
        PointCloud2 | None,
        ObstacleSegmentationMarkerArray | None,
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
        pcd_list: list[np.ndarray],
        header: Header,
        *,
        topic_rate: bool,
    ) -> tuple[dict, PointCloud2 | None, ObstacleSegmentationMarker | None]:
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
        map_to_baselink: dict,
        stop_reasons: list[dict],
        header: Header,
        *,
        topic_rate: bool,
    ) -> tuple[
        MarkerArray | None,
        PointCloud2 | None,
        ObstacleSegmentationMarkerArray | None,
        PointCloud2 | None,
        ObstacleSegmentationMarker | None,
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
        ) = self.summarize_detection(frame, header, topic_rate=topic_rate)
        (
            out_frame["NonDetection"],
            pcd_non_detection,
            graph_non_detection,
        ) = self.summarize_non_detection(
            frame.pointcloud_failed_non_detection,
            header,
            topic_rate=topic_rate,
        )
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
