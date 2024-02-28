# Copyright (c) 2024 TIER IV.inc
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

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

import driving_log_replayer.launch_common as cmn

RECORD_TOPIC_REGEX = """^/clock$\
|^/tf$\
|^/diagnostic/perception_online_evaluator/metrics\
"""


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    launch_arguments.append(DeclareLaunchArgument("sensing", default_value="false"))
    autoware_launch = cmn.get_autoware_launch(
        sensing=LaunchConfiguration("sensing"),
        localization="false",
    )
    rviz_node = cmn.get_rviz("annotation_less_perception.rviz")
    evaluator_node = cmn.get_evaluator_node("annotation_less_perception")

    player_normal = cmn.get_player(
        condition=UnlessCondition(LaunchConfiguration("sensing")),
    )

    player_remap = cmn.get_player(
        additional_argument=[
            "--remap",
            "/sensing/lidar/concatenated/pointcloud:=/driving_log_replayer/concatenated/pointcloud",
        ],
        condition=IfCondition(LaunchConfiguration("sensing")),
    )

    recorder, recorder_override = cmn.get_regex_recorders(
        "perception.qos.yaml",
        RECORD_TOPIC_REGEX,
    )
    return launch.LaunchDescription(
        [
            *launch_arguments,
            rviz_node,
            autoware_launch,
            evaluator_node,
            player_normal,
            player_remap,
            recorder,
            recorder_override,
        ],
    )
