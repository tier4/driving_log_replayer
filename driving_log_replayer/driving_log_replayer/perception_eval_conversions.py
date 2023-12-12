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

from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon as RosPolygon
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import Vector3
import numpy as np
from perception_eval.common.object import DynamicObject
from perception_eval.common.object import ObjectState
from perception_eval.evaluation.result.object_result import DynamicObjectWithPerceptionResult
from perception_eval.evaluation.result.perception_pass_fail_result import PassFailResult
from pyquaternion.quaternion import Quaternion
from rclpy.time import Duration
from shapely.geometry import Polygon
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def unix_time_from_ros_msg(ros_header: Header) -> int:
    return ros_header.stamp.sec * pow(10, 6) + ros_header.stamp.nanosec // 1000


def position_from_ros_msg(ros_position: Point) -> tuple[int, int, int]:
    return (ros_position.x, ros_position.y, ros_position.z)


def orientation_from_ros_msg(ros_orientation: RosQuaternion) -> Quaternion:
    return Quaternion(ros_orientation.w, ros_orientation.x, ros_orientation.y, ros_orientation.z)


def dimensions_from_ros_msg(
    ros_dimensions: Vector3,
    shape_type_num: int,
) -> tuple[float, float, float]:
    if shape_type_num == 1:
        # cylinder
        return (ros_dimensions.x, ros_dimensions.x, ros_dimensions.z)
    return (ros_dimensions.y, ros_dimensions.x, ros_dimensions.z)


def velocity_from_ros_msg(ros_velocity: Vector3) -> tuple[float, float, float]:
    return (ros_velocity.x, ros_velocity.y, ros_velocity.z)


def footprint_from_ros_msg(ros_footprint: RosPolygon) -> Polygon | None:
    coords = []
    for ros_point in ros_footprint.points:
        coords.append((ros_point.x, ros_point.y, ros_point.z))
    if len(coords) >= 3:  # noqa
        # polygon must be more than 3 points
        return Polygon(coords)
    # footprint.points of bounding_box and cylinder are empty, so return None
    return None


def uuid_from_ros_msg(ros_uuid: np.ndarray) -> str:
    """
    Convert uuid from unique_identifier_msgs.msg.UUID to string.

    Args:
    ----
        ros_uuid (np.ndarray): (16,) in uint8
    Returns:
        uuid (str)
    """
    return "".join([str(uuid_) for uuid_ in ros_uuid])


def object_state_to_ros_box_and_uuid(
    gt_state: ObjectState,
    header: Header,
    namespace: str,
    marker_id: int,
    color: ColorRGBA,
    text: str,
) -> tuple[Marker, Marker]:
    pose = Pose(
        position=Point(x=gt_state.position[0], y=gt_state.position[1], z=gt_state.position[2]),
        orientation=RosQuaternion(
            w=gt_state.orientation[0],
            x=gt_state.orientation[1],
            y=gt_state.orientation[2],
            z=gt_state.orientation[3],
        ),
    )
    # nuScenes box order is width, length, height
    scale = Vector3(x=gt_state.size[1], y=gt_state.size[0], z=gt_state.size[2])
    bbox = Marker(
        header=header,
        ns=namespace,
        id=marker_id,
        type=Marker.CUBE,
        action=Marker.ADD,
        lifetime=Duration(seconds=0.2).to_msg(),
        pose=pose,
        scale=scale,
        color=color,
    )
    uuid = Marker(
        header=header,
        ns=namespace + "_uuid",
        id=marker_id,
        type=Marker.TEXT_VIEW_FACING,
        action=Marker.ADD,
        lifetime=Duration(seconds=0.2).to_msg(),
        pose=pose,
        scale=Vector3(z=0.8),
        color=color,
        text=text[:8],
    )
    return bbox, uuid


def dynamic_objects_to_ros_points(
    container: list[DynamicObjectWithPerceptionResult] | list[DynamicObject],
    header: Header,
    scale: Vector3,
    color: ColorRGBA,
    namespace: str,
    marker_id: int,
    *,
    tp_gt: bool,
) -> Marker:
    points: list[Point] = []
    for obj in container:
        point = Point()
        if type(obj) == DynamicObjectWithPerceptionResult:
            if tp_gt:
                if obj.ground_truth_object is not None:
                    # tpのgtを出したい場合、tpならば必ずground_truthのペアがいる
                    point.x = obj.ground_truth_object.state.position[0]
                    point.y = obj.ground_truth_object.state.position[1]
                    point.z = obj.ground_truth_object.state.position[2]
            else:
                point.x = obj.estimated_object.state.position[0]
                point.y = obj.estimated_object.state.position[1]
                point.z = obj.estimated_object.state.position[2]
        if type(obj) == DynamicObject:
            point.x = obj.state.position[0]
            point.y = obj.state.position[1]
            point.z = obj.state.position[2]
        points.append(point)
    return Marker(
        header=header,
        ns=namespace,
        id=marker_id,
        type=Marker.POINTS,
        action=Marker.ADD,
        lifetime=Duration(seconds=0.2).to_msg(),
        scale=scale,
        color=color,
        points=points,
    )


def pass_fail_result_to_ros_points_array(pass_fail: PassFailResult, header: Header) -> MarkerArray:
    marker_results = MarkerArray()

    scale = Vector3(x=1.0, y=1.0, z=0.0)

    if objs := pass_fail.tp_object_results:
        # estimated obj
        marker = dynamic_objects_to_ros_points(
            objs,
            header,
            scale,
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
            "tp_est",
            0,
            tp_gt=False,
        )
        marker_results.markers.append(marker)
    if objs := pass_fail.tp_object_results:
        # ground truth obj
        marker = dynamic_objects_to_ros_points(
            objs,
            header,
            scale,
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            "tp_gt",
            0,
            tp_gt=True,
        )
        marker_results.markers.append(marker)
    if objs := pass_fail.fp_object_results:
        marker = dynamic_objects_to_ros_points(
            objs,
            header,
            scale,
            ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
            "fp",
            0,
            tp_gt=False,
        )
        marker_results.markers.append(marker)
    if objs := pass_fail.fn_objects:
        marker = dynamic_objects_to_ros_points(
            objs,
            header,
            scale,
            ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
            "fn",
            0,
            tp_gt=False,
        )
        marker_results.markers.append(marker)
    return marker_results
