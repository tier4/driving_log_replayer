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
from launch_ros.descriptions import ParameterValue

import driving_log_replayer.launch_common as cmn

RECORD_TOPIC_REGEX = """^/clock$\
|^/tf$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/perception/perception_online_evaluator/.*\
"""


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    launch_arguments.append(DeclareLaunchArgument("sensing", default_value="false"))
    launch_arguments.append(
        DeclareLaunchArgument("annotationless_threshold_file", default_value=""),
    )
    launch_arguments.append(
        DeclareLaunchArgument("annotationless_pass_range", default_value=""),
    )
    autoware_launch = cmn.get_autoware_launch(
        sensing=LaunchConfiguration("sensing"),
        localization="false",
        use_perception_online_evaluator="true",
    )
    evaluator_node = cmn.get_evaluator_node(
        "annotationless_perception",
        addition_parameter={
            "annotationless_threshold_file": LaunchConfiguration("annotationless_threshold_file"),
            # annotationless_pass_range is json format string. Avoid interpreting json format strings as dict
            # [ERROR] [launch]: Caught exception in launch (see debug for traceback): Allowed value types are bytes, bool, int, float, str, Sequence[bool], Sequence[int], Sequence[float], Sequence[str].
            # Got <class 'dict'>.If the parameter is meant to be a string, try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
            "annotationless_pass_range": ParameterValue(
                LaunchConfiguration("annotationless_pass_range"),
                value_type=str,
            ),
        },
    )

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
            autoware_launch,
            evaluator_node,
            player_normal,
            player_remap,
            recorder,
            recorder_override,
        ],
    )
