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

from typing import List
from typing import Tuple
from typing import Union

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import Vector3
from perception_eval.common.object import DynamicObject
from perception_eval.common.object import ObjectState
from perception_eval.evaluation.result.object_result import DynamicObjectWithPerceptionResult
from perception_eval.evaluation.result.perception_pass_fail_result import PassFailResult
from pyquaternion.quaternion import Quaternion
from rclpy.time import Duration
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def unix_time_from_ros_msg(ros_header: Header) -> int:
    return ros_header.stamp.sec * pow(10, 6) + ros_header.stamp.nanosec // 1000


def position_from_ros_msg(ros_position: Point) -> Tuple[int, int, int]:
    return (ros_position.x, ros_position.y, ros_position.z)


def orientation_from_ros_msg(ros_orientation: RosQuaternion) -> Quaternion:
    return Quaternion(ros_orientation.w, ros_orientation.x, ros_orientation.y, ros_orientation.z)


def dimensions_from_ros_msg(ros_dimensions: Vector3) -> Tuple[float, float, float]:
    return (ros_dimensions.y, ros_dimensions.x, ros_dimensions.z)


def velocity_from_ros_msg(ros_velocity: Vector3) -> Tuple[float, float, float]:
    return (ros_velocity.x, ros_velocity.y, ros_velocity.z)


def uuid_from_ros_msg(ros_uuid) -> str:
    """Convert uuid from unique_identifier_msgs.msg.UUID to string.

    Args:
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
) -> Tuple[Marker, Marker]:
    pose = Pose()
    pose.position.x = gt_state.position[0]
    pose.position.y = gt_state.position[1]
    pose.position.z = gt_state.position[2]
    pose.orientation.w = gt_state.orientation[0]
    pose.orientation.x = gt_state.orientation[1]
    pose.orientation.y = gt_state.orientation[2]
    pose.orientation.z = gt_state.orientation[3]
    scale = Vector3()
    # nuScenes box order is width, length, height
    scale.x = gt_state.size[1]
    scale.y = gt_state.size[0]
    scale.z = gt_state.size[2]
    bbox = Marker()
    bbox.header = header
    bbox.ns = namespace
    bbox.id = marker_id
    bbox.type = Marker.CUBE
    bbox.action = Marker.ADD
    bbox.lifetime = Duration(seconds=0.2).to_msg()
    bbox.pose = pose
    bbox.scale = scale
    bbox.color = color
    uuid = Marker()
    uuid.header = header
    uuid.ns = namespace + "_uuid"
    uuid.id = marker_id
    uuid.type = Marker.TEXT_VIEW_FACING
    uuid.action = Marker.ADD
    uuid.lifetime = Duration(seconds=0.2).to_msg()
    uuid_scale = Vector3()
    uuid_scale.z = 0.8
    uuid.pose = pose
    uuid.scale = uuid_scale
    uuid.color = color
    uuid.text = text[:8]
    return bbox, uuid


def dynamic_objects_to_ros_points(
    container: Union[List[DynamicObjectWithPerceptionResult], List[DynamicObject]],
    header: Header,
    scale: Vector3,
    color: ColorRGBA,
    namespace: str,
    marker_id: int,
    tp_gt: bool = False,
):
    p_marker = Marker()
    p_marker.header = header
    p_marker.ns = namespace
    p_marker.id = marker_id
    p_marker.type = Marker.POINTS
    p_marker.action = Marker.ADD
    p_marker.lifetime = Duration(seconds=0.2).to_msg()
    p_marker.scale = scale
    p_marker.color = color

    for obj in container:
        point = Point()
        if type(obj) == DynamicObjectWithPerceptionResult:
            if tp_gt:
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
        p_marker.points.append(point)
    return p_marker


def pass_fail_result_to_ros_points_array(pass_fail: PassFailResult, header: Header) -> MarkerArray:
    marker_results = MarkerArray()

    scale = Vector3(x=1.0, y=1.0, z=0.0)

    if objs := pass_fail.tp_objects:
        # estimated obj
        c_tp_est = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        marker = dynamic_objects_to_ros_points(objs, header, scale, c_tp_est, "tp_est", 0)
        marker_results.markers.append(marker)
    if objs := pass_fail.tp_objects:
        # ground truth obj
        c_tp_gt = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker = dynamic_objects_to_ros_points(objs, header, scale, c_tp_gt, "tp_gt", 0, tp_gt=True)
        marker_results.markers.append(marker)
    if objs := pass_fail.fp_objects_result:
        c_fp = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        marker = dynamic_objects_to_ros_points(objs, header, scale, c_fp, "fp", 0)
        marker_results.markers.append(marker)
    if objs := pass_fail.fn_objects:
        c_fn = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        marker = dynamic_objects_to_ros_points(objs, header, scale, c_fn, "fn", 0)
        marker_results.markers.append(marker)
    return marker_results
