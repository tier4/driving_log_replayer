# Copyright (c) 2023 TIER IV.inc
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

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon as RosPolygon
from geometry_msgs.msg import Quaternion as RosQuaternion
from pyquaternion.quaternion import Quaternion
from shapely.geometry import Polygon
from std_msgs.msg import Header

from log_evaluator.perception_eval_conversions import footprint_from_ros_msg
from log_evaluator.perception_eval_conversions import orientation_from_ros_msg
from log_evaluator.perception_eval_conversions import position_from_ros_msg
from log_evaluator.perception_eval_conversions import unix_time_from_ros_msg


def test_unix_time_from_ros_msg() -> None:
    unix_time = unix_time_from_ros_msg(Header(stamp=Time(sec=1234567890, nanosec=123456789)))
    assert unix_time == 1234567890123456  # noqa


def test_position_from_ros_msg() -> None:
    tuple_position = position_from_ros_msg(Point(x=1.0, y=2.0, z=3.0))
    assert tuple_position == (1.0, 2.0, 3.0)


def test_orientation_from_ros_msg() -> None:
    eval_quaternion = orientation_from_ros_msg(RosQuaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    assert eval_quaternion == Quaternion(1.0, 0.0, 0.0, 0.0)


def test_footprint_from_ros_msg_normal() -> None:
    ros_points = [
        Point32(x=0.0, y=0.0, z=0.0),
        Point32(x=1.0, y=1.0, z=1.0),
        Point32(x=2.0, y=2.0, z=2.0),
    ]
    eval_polygon = footprint_from_ros_msg(RosPolygon(points=ros_points))
    coords = ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0))
    assert eval_polygon == Polygon(coords)


def test_footprint_from_ros_msg_invalid_footprint() -> None:
    ros_points = [
        Point32(x=0.0, y=0.0, z=0.0),
        Point32(x=1.0, y=1.0, z=1.0),
    ]
    eval_polygon = footprint_from_ros_msg(RosPolygon(points=ros_points))
    assert eval_polygon is None


def test_footprint_from_ros_msg_empty_footprint() -> None:
    ros_points = []
    eval_polygon = footprint_from_ros_msg(RosPolygon(points=ros_points))
    assert eval_polygon is None
