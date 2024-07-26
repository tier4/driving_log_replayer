# Copyright (c) 2024 TierIV.inc
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
from launch.substitutions import LaunchConfiguration

import driving_log_replayer.launch_common as cmn

RECORD_TOPIC_REGEX = """^/clock$\
|^/tf$\
|^/driving_log_replayer/.*\
"""


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    launch_arguments.append(
        DeclareLaunchArgument(
            "evaluation_target_topic",
            default_value="/perception/obstacle_segmentation/pointcloud",
        ),
    )
    autoware_launch = cmn.get_autoware_launch(
        sensing="false",
        localization="false",
        planning="false",
        control="false",
        scenario_simulation="true",
        perception_mode="lidar",
    )

    evaluator_node = cmn.get_evaluator_node(
        "ground_segmentation",
        addition_parameter={
            "vehicle_model": LaunchConfiguration("vehicle_model"),
            "evaluation_target_topic": LaunchConfiguration("evaluation_target_topic"),
        },
    )
    player = cmn.get_player()

    recorder, recorder_override = cmn.get_regex_recorders(
        "obstacle_segmentation.qos.yaml",
        RECORD_TOPIC_REGEX,
    )

    return launch.LaunchDescription(
        [
            *launch_arguments,
            autoware_launch,
            evaluator_node,
            player,
            recorder,
            recorder_override,
        ],
    )
