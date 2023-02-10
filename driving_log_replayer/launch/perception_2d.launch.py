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

import copy

import driving_log_replayer.launch_common
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_arguments = driving_log_replayer.launch_common.get_driving_log_replayer_common_argument()
    launch_arguments.append(DeclareLaunchArgument("sensing", default_value="false"))
    autoware_launch = driving_log_replayer.launch_common.get_autoware_launch(
        sensing=LaunchConfiguration("sensing"), localization="false"
    )
    rviz_node = driving_log_replayer.launch_common.get_rviz("perception_2d.rviz")
    evaluator_node = driving_log_replayer.launch_common.get_evaluator_node(
        "perception_2d", python_node=True
    )
    evaluator_shutdown = driving_log_replayer.launch_common.get_evaluator_shutdown(evaluator_node)

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
    play_cmd_remap.extend(
        [
            "--remap",
            "/sensing/lidar/concatenated/pointcloud:=/driving_log_replayer/concatenated/pointcloud",
        ]
    )

    player_normal = ExecuteProcess(
        cmd=["sleep", LaunchConfiguration("play_delay")],
        on_exit=[ExecuteProcess(cmd=play_cmd)],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("sensing")),
    )

    player_remap = ExecuteProcess(
        cmd=["sleep", LaunchConfiguration("play_delay")],
        on_exit=[ExecuteProcess(cmd=play_cmd_remap)],
        output="screen",
        condition=IfCondition(LaunchConfiguration("sensing")),
    )

    recorder = driving_log_replayer.launch_common.get_regex_recorder(
        "perception.qos.yaml",
        "^/clock$|^/tf$|/sensing/lidar/concatenated/pointcloud|^/perception/object_recognition/detection/objects$|^/perception/object_recognition/tracking/objects$|^/perception/object_recognition/objects$|^/driving_log_replayer/.*|^/sensing/camera/.*",
    )

    return launch.LaunchDescription(
        launch_arguments
        + [
            rviz_node,
            autoware_launch,
            evaluator_node,
            player_normal,
            player_remap,
            recorder,
            evaluator_shutdown,
        ]
    )
