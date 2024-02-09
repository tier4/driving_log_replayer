# Copyright (c) 2021 TierIV.inc
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
from launch.substitutions import LaunchConfiguration

import driving_log_replayer.launch_common as cmn

RECORD_TOPIC_REGEX = """^/clock$\
|^/tf$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/planning/scenario_planning/trajectory$\
|^/planning/scenario_planning/status/stop_reasons$\
|^/driving_log_replayer/.*\
"""


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    autoware_launch = cmn.get_autoware_launch(
        planning="true",
        localization="false",
        control="true",
        scenario_simulation="true",
    )
    rviz_node = cmn.get_rviz("obstacle_segmentation.rviz")
    evaluator_node = cmn.get_evaluator_node(
        "obstacle_segmentation",
        addition_parameter={
            "vehicle_model": LaunchConfiguration("vehicle_model"),
            "map_path": LaunchConfiguration("map_path"),
        },
    )
    player = cmn.get_player(
        additional_argument=[
            "--remap",
            "/sensing/lidar/concatenated/pointcloud:=/driving_log_replayer/unused_concatenated_pointcloud",
        ],
    )

    recorder, recorder_override = cmn.get_regex_recorders(
        "obstacle_segmentation.qos.yaml",
        RECORD_TOPIC_REGEX,
    )

    return launch.LaunchDescription(
        [
            *launch_arguments,
            rviz_node,
            autoware_launch,
            evaluator_node,
            player,
            recorder,
            recorder_override,
        ],
    )
