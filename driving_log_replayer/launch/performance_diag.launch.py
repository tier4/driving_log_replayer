# Copyright (c) 2022 TIER IV.inc
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

import copy

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

import driving_log_replayer.launch_common as cmn

RECORD_TOPIC_REGEX = """^/clock$\
|^/tf$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/diagnostics$\
|^/sensing/lidar/.*/blockage_diag/debug/blockage_mask_image$\
|^/driving_log_replayer/.*\
"""


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    launch_arguments.append(DeclareLaunchArgument("localization", default_value="false"))
    fitter_launch = cmn.get_map_height_fitter(
        launch_service=LaunchConfiguration("localization"),
    )
    autoware_launch = cmn.get_autoware_launch(
        localization=LaunchConfiguration("localization"),
    )
    rviz_node = cmn.get_rviz("autoware.rviz")
    evaluator_node = cmn.get_evaluator_node(
        "performance_diag",
        addition_parameter={"localization": LaunchConfiguration("localization")},
    )

    play_cmd = [
        "ros2",
        "bag",
        "play",
        LaunchConfiguration("input_bag"),
        "--rate",
        LaunchConfiguration("play_rate"),
        "--clock",
        "200",
    ]
    play_cmd_remap = copy.copy(play_cmd)
    play_cmd_remap.extend(["--remap", "/tf:=/driving_log_replayer_unused/tf"])

    player_normal = ExecuteProcess(
        cmd=["sleep", LaunchConfiguration("play_delay")],
        on_exit=[ExecuteProcess(cmd=play_cmd)],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("localization")),
    )

    player_remap = ExecuteProcess(
        cmd=["sleep", LaunchConfiguration("play_delay")],
        on_exit=[ExecuteProcess(cmd=play_cmd_remap)],
        output="screen",
        condition=IfCondition(LaunchConfiguration("localization")),
    )

    recorder, recorder_override = cmn.get_regex_recorders(
        "performance_diag.qos.yaml",
        RECORD_TOPIC_REGEX,
    )
    return launch.LaunchDescription(
        [
            *launch_arguments,
            rviz_node,
            autoware_launch,
            fitter_launch,
            evaluator_node,
            player_normal,
            player_remap,
            recorder,
            recorder_override,
        ],
    )
