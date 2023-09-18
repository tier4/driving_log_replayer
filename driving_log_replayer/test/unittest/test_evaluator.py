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
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
import pytest
from std_msgs.msg import Header

from driving_log_replayer.evaluator import DLREvaluator


class SampleEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name)

    def check_scenario(self) -> None:
        pass


@pytest.fixture()
def create_sample_evaluator() -> DLREvaluator:
    return SampleEvaluator("sample_evaluator")


# def test_transform_stamped_with_euler_angle() -> None:
#     tf = TransformStamped(
#         header=Header(stamp=Time(sec=123, nanosec=456), frame_id="map"),
#         child_frame_id="base_link",
#         transform=Transform(
#             translation=Vector3(x=3837.475624794553, y=73730.40550344331, z=19.638391309049716),
#             rotation=Quaternion(
#                 x=0.023889448655962233,
#                 y=-0.006285222290687734,
#                 z=0.9756628880587501,
#                 w=-0.21788005665624788,
#             ),
#         ),
#     )
#     dict_tf_euler = {
#         "TransformStamped": {
#             "header": {
#                 "stamp": {
#                     "sec": 123,
#                     "nanosec": 456,
#                 },
#                 "frame_id": "map",
#             },
#             "child_frame_id": "base_link",
#             "transform": {
#                 "translation": {
#                     "x": 3837.475624794553,
#                     "y": 73730.40550344331,
#                     "z": 19.638391309049716,
#                 },
#                 "rotation": {
#                     "x": 0.023889448655962233,
#                     "y": -0.006285222290687734,
#                     "z": 0.9756628880587501,
#                     "w": -0.21788005665624788,
#                 },
#             },
#             "rotation_euler": {
#                 "roll": -0.022698392435205317,
#                 "pitch": -0.04389133881352741,
#                 "yaw": -2.7016745033573883,
#             },
#         },
#     }
#     assert DLREvaluator.transform_stamped_with_euler_angle(tf) == dict_tf_euler
