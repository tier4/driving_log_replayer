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

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Literal

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
import numpy as np
from perception_eval.common.object import DynamicObject
from perception_eval.evaluation.sensing.sensing_frame_config import SensingFrameConfig
from perception_eval.evaluation.sensing.sensing_frame_result import SensingFrameResult
from perception_eval.evaluation.sensing.sensing_result import DynamicObjectWithSensingResult
from pydantic import BaseModel
from pydantic import conlist
from pydantic import field_validator
import ros2_numpy
from rosidl_runtime_py import message_to_ordereddict
from sensor_msgs.msg import PointCloud2
from shapely.geometry import Polygon
import simplejson as json
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import yaml

import log_evaluator.perception_eval_conversions as eval_conversions
from log_evaluator.result import EvaluationItem
from log_evaluator.result import ResultBase
from log_evaluator.scenario import number
from log_evaluator.scenario import Scenario
from log_evaluator_analyzer.config.obstacle_segmentation import Config
from log_evaluator_analyzer.config.obstacle_segmentation import load_config
from log_evaluator_analyzer.data import DistType
from log_evaluator_analyzer.data.obstacle_segmentation import fail_3_times_in_a_row
from log_evaluator_analyzer.data.obstacle_segmentation import JsonlParser
from log_evaluator_analyzer.plot import PlotBase
from log_evaluator_msgs.msg import ObstacleSegmentationMarker
from log_evaluator_msgs.msg import ObstacleSegmentationMarkerArray


def get_goal_pose_from_t4_dataset(dataset_path: str) -> PoseStamped:
    ego_pose_json_path = Path(dataset_path, "annotation", "ego_pose.json")
    with ego_pose_json_path.open() as ego_pose_file:
        ego_pose_json = json.load(ego_pose_file)
        last_ego_pose = ego_pose_json[-1]
        pose = Pose(
            position=Point(
                x=last_ego_pose["translation"][0],
                y=last_ego_pose["translation"][1],
                z=last_ego_pose["translation"][2],
            ),
            orientation=Quaternion(
                x=last_ego_pose["rotation"][1],
                y=last_ego_pose["rotation"][2],
                z=last_ego_pose["rotation"][3],
                w=last_ego_pose["rotation"][0],
            ),
        )
        return PoseStamped(header=Header(frame_id="map"), pose=pose)


class ProposedAreaCondition(BaseModel):
    polygon_2d: list[conlist(number, min_length=2, max_length=2)]
    z_min: number
    z_max: number

    @field_validator("polygon_2d")
    @classmethod
    def is_clockwise(cls, v: list[list[number]]) -> list | None:
        v_float = []
        if len(v) < 3:  # noqa
            err_msg = "polygon requires 3 or more elements"
            raise ValueError(err_msg)
        check_clock_wise: float = 0.0
        # convert int value to float
        for p_2d in v:
            v_float.append([float(p_2d[0]), float(p_2d[1])])
        for i, _ in enumerate(v_float):
            p1 = v_float[i]
            p2 = v_float[(i + 1) % len(v_float)]
            check_clock_wise += (p2[0] - p1[0]) * (p2[1] + p2[1])
        if check_clock_wise <= 0.0:  # noqa
            err_msg = "polygon_2d is not clockwise"
            raise ValueError(err_msg)
        return v_float

    @field_validator("z_min", "z_max")
    @classmethod
    def to_float(cls, v: number) -> float:
        return float(v)

    def search_range(self) -> float:
        coord: list = []
        for p in self.polygon_2d:
            coord.append([p[0], p[1]])
        poly = Polygon(coord)
        min_x, min_y, max_x, max_y = poly.bounds
        abs_max_x = max(abs(min_x), abs(max_x))
        abs_max_y = max(abs(min_y), abs(max_y))
        return np.sqrt(abs_max_x**2 + abs_max_y**2)


class BoundingBoxCondition(BaseModel):
    Start: number | None = None
    End: number | None = None


class DetectionCondition(BaseModel):
    PassRate: number
    BoundingBoxConfig: list[dict[str, BoundingBoxCondition]] | None


class NonDetectionCondition(BaseModel):
    PassRate: number
    ProposedArea: ProposedAreaCondition


class Conditions(BaseModel):
    Detection: DetectionCondition | None
    NonDetection: NonDetectionCondition | None


class Evaluation(BaseModel):
    UseCaseName: Literal["obstacle_segmentation"]
    UseCaseFormatVersion: Literal["0.3.0"]
    Datasets: list[dict]
    Conditions: Conditions
    SensingEvaluationConfig: dict


class ObstacleSegmentationScenario(Scenario):
    Evaluation: Evaluation


def default_config_path() -> Path:
    return Path(
        get_package_share_directory("log_evaluator_analyzer"),
        "config",
        "obstacle_segmentation.yaml",
    )


def update_config(config: Config, vehicle_model: str) -> Config:
    vehicle_info_param = Path(
        get_package_share_directory(vehicle_model + "_description"),
        "config",
        "vehicle_info.param.yaml",
    )
    with vehicle_info_param.open() as f:
        yaml_obj = yaml.safe_load(f)

    config.overhang_from_baselink = (
        yaml_obj["/**"]["ros__parameters"]["front_overhang"]
        + yaml_obj["/**"]["ros__parameters"]["wheel_base"]
    )
    return config


def get_graph_data(
    input_jsonl: Path,
    vehicle_model: str,
    output_dir: Path,
    config_yaml: Path,
    dist_type: DistType,
) -> tuple[dict, dict]:
    output_dir.mkdir(exist_ok=True)

    # 設定ファイルのロード
    config = load_config(config_yaml)
    # ここにvehicle paramから更新するところ入れる
    config = update_config(config, vehicle_model)

    # Load result.jsonl
    parser = JsonlParser(input_jsonl, config, dist_type)

    detection_dist_plot = PlotBase()
    detection_dist_plot.add_data(parser.get_bb_distance(), legend="1 Frame")

    min3_data = fail_3_times_in_a_row(parser.get_bb_distance())
    detection_dist_plot.add_data(min3_data, legend="3 Frames")

    pointcloud_numpoints_plot = PlotBase()
    for data in parser.get_pointcloud_points_per_uuid():
        pointcloud_numpoints_plot.add_data(data, legend=data[0][2])

    return detection_dist_plot.to_dict(), pointcloud_numpoints_plot.to_dict()


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
    container_type: str,
    header: Header,
    color: ColorRGBA,
    info: dict,
    counter: int,
    marker_array: MarkerArray,
    pcd: np.ndarray,
    graph_detection: ObstacleSegmentationMarkerArray,
    result_status: int,
) -> int:
    for result in container:
        pcd_result = np.zeros(
            result.inside_pointcloud_num,
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        pcd_result["x"] = result.inside_pointcloud[:, 0]
        pcd_result["y"] = result.inside_pointcloud[:, 1]
        pcd_result["z"] = result.inside_pointcloud[:, 2]
        pcd = np.concatenate([pcd, pcd_result])

        bbox, uuid, info[container_type] = get_box_marker(
            result.ground_truth_object,
            header,
            "detection",
            counter := counter
            + 1,  # アノテーションは基本的にどのフレームでもあるので、uuidで指定した分だけ全体でカウンターが回る。
            color,
        )
        marker_array.markers.append(bbox)
        marker_array.markers.append(uuid)
        info[container_type]["PointCloud"] = {
            "NumPoints": result.inside_pointcloud_num,
            "Nearest": result.nearest_point.tolist() if result.nearest_point is not None else [],
            "Stamp": message_to_ordereddict(header.stamp),
        }
        marker = ObstacleSegmentationMarker(
            header=header,
            number_of_pointcloud=result.inside_pointcloud_num,
            status=result_status,
            bbox_uuid=result.ground_truth_object.uuid,
        )
        graph_detection.markers.append(marker)
    return counter


def get_sensing_frame_config(
    pcd_header: Header,
    scenario: ObstacleSegmentationScenario,
) -> tuple[bool, SensingFrameConfig | None]:
    detection_config = scenario.Evaluation.Conditions.Detection
    if detection_config is None:
        return True, None
    bbox_conf = detection_config.BoundingBoxConfig
    if bbox_conf is None:
        return True, None
    target_uuids = []
    for uuid_dict in bbox_conf:
        for uuid, v in uuid_dict.items():
            # uuid: str, v: Dict
            start = v.Start
            end = v.End
            if start is None:
                start = 0.0
            if end is None:
                end = sys.float_info.max
            stamp_float = pcd_header.stamp.sec + (pcd_header.stamp.nanosec / pow(10, 9))
            if start <= stamp_float <= end:
                target_uuids.append(uuid)
    if target_uuids == []:
        return False, None
    e_conf = scenario.Evaluation.SensingEvaluationConfig["evaluation_config_dict"]
    sensing_frame_config = SensingFrameConfig(
        target_uuids=target_uuids,
        box_scale_0m=e_conf["box_scale_0m"],
        box_scale_100m=e_conf["box_scale_100m"],
        min_points_threshold=e_conf["min_points_threshold"],
    )
    return True, sensing_frame_config


def get_proposed_area_list_point_stamped(
    proposed_area: ProposedAreaCondition,
    header: Header,
) -> list[PointStamped]:
    points: list[PointStamped] = []
    for point in proposed_area.polygon_2d:
        points.append(PointStamped(header=header, point=Point(x=point[0], y=point[1], z=0.0)))
    return points


def list_point_stamped_to_line_strip(
    list_point_stamped: list[PointStamped],
    marker_id: int,
) -> Marker:
    line_strip = Marker(
        header=list_point_stamped[0].header,
        type=Marker.LINE_STRIP,
        action=Marker.ADD,
        ns="intersection",
        id=marker_id,
        color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3, x=0.2, y=0.2, z=0.2),
    )
    for point_stamped in list_point_stamped:
        line_strip.points.append(point_stamped.point)
    return line_strip


def transform_proposed_area(
    proposed_area: ProposedAreaCondition,
    header: Header,
    tf: TransformStamped,
) -> tuple[Polygon, float]:
    proposed_area_list: list = []
    sum_z = 0.0
    count_point = 0

    list_point_stamped = get_proposed_area_list_point_stamped(proposed_area, header)
    for point_stamped in list_point_stamped:
        count_point += 1
        point_stamped_in_map = do_transform_point(point_stamped, tf)
        proposed_area_list.append([point_stamped_in_map.point.x, point_stamped_in_map.point.y])
        sum_z += point_stamped_in_map.point.z
    average_z = sum_z / count_point
    return Polygon(proposed_area_list), average_z


def get_non_detection_area_in_base_link(
    intersection_polygon: Polygon,
    header: Header,
    z_min: float,
    z_max: float,
    average_z: float,
    base_link_to_map: TransformStamped,
    marker_id: int,
) -> tuple[Marker, list]:
    line_strip = Marker(
        header=header,
        type=Marker.LINE_STRIP,
        action=Marker.ADD,
        ns="intersection",
        id=marker_id,
        color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3),
        scale=Vector3(x=0.2, y=0.2, z=0.2),
        lifetime=Duration(nanosec=200_000_000),
    )
    list_intersection_area = []
    list_p_stamped_base_link: list[PointStamped] = []
    for i, shapely_point in enumerate(intersection_polygon.exterior.coords):
        if i != len(intersection_polygon.exterior.coords) - 1:
            p_stamped_map = PointStamped(
                header=header,
                point=Point(x=shapely_point[0], y=shapely_point[1], z=average_z),
            )
            list_p_stamped_base_link.append(do_transform_point(p_stamped_map, base_link_to_map))
    # create floor polygon
    for p_base_link in list_p_stamped_base_link:
        p_base_link.point.z = z_min
        line_strip.points.append(deepcopy(p_base_link.point))
        list_intersection_area.append(
            [p_base_link.point.x, p_base_link.point.y, p_base_link.point.z],
        )
    # create roof polygon
    for p_base_link in list_p_stamped_base_link:
        p_base_link.point.z = z_max
        line_strip.points.append(p_base_link.point)
        list_intersection_area.append(
            [p_base_link.point.x, p_base_link.point.y, p_base_link.point.z],
        )
    return line_strip, list_intersection_area


def set_ego_point(map_to_baselink: TransformStamped) -> Point:
    t = map_to_baselink.transform.translation
    return Point(x=t.x, y=t.y, z=t.z)


@dataclass
class Detection(EvaluationItem):
    name: str = "Detection"
    warn: int = 0

    def set_frame(
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
        if self.condition is None:
            self.success = True
            self.summary = "Invalid"
            return (
                {"Result": {"Total": self.success_str(), "Frame": "Invalid"}, "Info": {}},
                None,
                None,
                None,
            )
        self.condition: DetectionCondition
        self.total += 1
        frame_success = "Fail"
        if len(frame_result.detection_warning_results) != 0:
            frame_success = "Warn"
            self.warn += 1
        elif (
            len(frame_result.detection_fail_results) == 0
            and len(frame_result.detection_success_results) != 0
            and topic_rate
        ):
            frame_success = "Success"
            self.passed += 1

        current_rate = self.rate()
        self.success = current_rate >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {current_rate:.2f}% (Warn: {self.warn})"

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
            if frame_success == "Success"
            else ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
        )
        info = {}
        counter = summarize_frame_container(
            frame_result.detection_success_results,
            "DetectionSuccess",
            header,
            color_success,
            info,
            counter,
            marker_array,
            pcd,
            graph_detection,
            (
                ObstacleSegmentationMarker.OK
                if frame_success == "Success"
                else ObstacleSegmentationMarker.ERROR
            ),
        )

        color_fail = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
        counter = summarize_frame_container(
            frame_result.detection_fail_results,
            "DetectionFail",
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
        counter = summarize_frame_container(
            frame_result.detection_warning_results,
            "DetectionWarn",
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
        return (
            {"Result": {"Total": self.success_str(), "Frame": frame_success}, "Info": info},
            marker_array,
            pcd_detection,
            graph_detection,
        )


@dataclass
class NonDetection(EvaluationItem):
    name: str = "NonDetection"

    def set_frame(
        self,
        pcd_list: list[np.ndarray],
        header: Header,
        *,
        topic_rate: bool,
    ) -> tuple[dict, PointCloud2 | None, ObstacleSegmentationMarker | None]:
        if self.condition is None:
            self.success = True
            self.summary = "Invalid"
            return (
                {"Result": {"Total": self.success_str(), "Frame": "Invalid"}, "Info": {}},
                None,
                None,
            )
        self.condition: NonDetectionCondition

        self.total += 1
        frame_success = "Fail"
        if len(pcd_list) == 0 and topic_rate:
            frame_success = "Success"
            self.passed += 1

        current_rate = self.rate()
        self.success = current_rate >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {current_rate:.2f}%"

        ros_pcd, dist_array = NonDetection.convert_numpy_pointcloud(pcd_list, header)
        graph_non_detection = ObstacleSegmentationMarker(
            header=header,
            status=(
                ObstacleSegmentationMarker.ERROR
                if frame_success == "Fail"
                else ObstacleSegmentationMarker.OK
            ),
            number_of_pointcloud=dist_array.shape[0],
        )
        info = {}
        if len(pcd_list) != 0:  # Failだとtopic_rateが原因の場合もある
            # 詳細情報の追記を書く
            dist_dict = {}
            for i in range(100):
                dist_dict[f"{i}-{i+1}"] = np.count_nonzero(
                    (i <= dist_array) & (dist_array < i + 1),
                )
            info = {
                "PointCloud": {"NumPoints": dist_array.shape[0], "Distance": dist_dict},
            }

        return (
            {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": info,
            },
            ros_pcd,
            graph_non_detection,
        )

    @classmethod
    def convert_numpy_pointcloud(
        cls,
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


class ObstacleSegmentationResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__detection = Detection(condition=condition.Detection)
        self.__non_detection = NonDetection(condition=condition.NonDetection)

    def update(self) -> None:
        summary_str = f"{self.__detection.summary}, {self.__non_detection.summary}"
        if self.__detection.success and self.__non_detection.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

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
            "StopReasons": stop_reasons,
            "TopicRate": str(topic_rate),
        }
        (
            out_frame["Detection"],
            marker_detection,
            pcd_detection,
            graph_detection,
        ) = self.__detection.set_frame(frame, header, topic_rate=topic_rate)
        (
            out_frame["NonDetection"],
            pcd_non_detection,
            graph_non_detection,
        ) = self.__non_detection.set_frame(
            frame.pointcloud_failed_non_detection,
            header,
            topic_rate=topic_rate,
        )
        self._frame = out_frame
        self.update()
        return (
            marker_detection,
            pcd_detection,
            graph_detection,
            pcd_non_detection,
            graph_non_detection,
        )
