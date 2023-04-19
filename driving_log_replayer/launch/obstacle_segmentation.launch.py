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

import driving_log_replayer.launch_common
import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_arguments = driving_log_replayer.launch_common.get_driving_log_replayer_common_argument()
    autoware_launch = driving_log_replayer.launch_common.get_autoware_launch(
        planning="true", localization="false", control="true", scenario_simulation="true"
    )
    rviz_node = driving_log_replayer.launch_common.get_rviz("obstacle_segmentation.rviz")
    evaluator_node = driving_log_replayer.launch_common.get_evaluator_node(
        "obstacle_segmentation",
        python_node=True,
        addition_parameter={"vehicle_model": LaunchConfiguration("vehicle_model")},
    )
    evaluator_shutdown = driving_log_replayer.launch_common.get_evaluator_shutdown(evaluator_node)
    evaluator_sub_node = Node(
        package="driving_log_replayer",
        namespace="/driving_log_replayer",
        executable="obstacle_segmentation_evaluator_node",
        output="screen",
        name="obstacle_segmentation_sub",
        parameters=[{"use_sim_time": True, "scenario_path": LaunchConfiguration("scenario_path")}],
    )
    evaluator_sub_shutdown = driving_log_replayer.launch_common.get_evaluator_shutdown(
        evaluator_sub_node
    )
    player = driving_log_replayer.launch_common.get_player(
        additional_argument=[
            "--remap",
            "/sensing/lidar/concatenated/pointcloud:=/driving_log_replayer/unused_concatenated_pointcloud",
        ]
    )

    recorder = driving_log_replayer.launch_common.get_recorder(
        "obstacle_segmentation.qos.yaml",
        [
            "/clock",
            "/tf",
            "/perception/obstacle_segmentation/pointcloud",
            "/planning/scenario_planning/status/stop_reasons",
            "/planning/scenario_planning/trajectory",
            "/driving_log_replayer/marker/detection",
            "/driving_log_replayer/marker/non_detection",
            "/driving_log_replayer/pcd/detection",
            "/driving_log_replayer/pcd/non_detection",
            "/driving_log_replayer/obstacle_segmentation/input",
            "/driving_log_replayer/graph/topic_rate",
            "/driving_log_replayer/graph/detection",
            "/driving_log_replayer/graph/non_detection",
        ],
    )

    return launch.LaunchDescription(
        launch_arguments
        + [
            rviz_node,
            autoware_launch,
            evaluator_node,
            evaluator_sub_node,
            player,
            recorder,
            evaluator_shutdown,
            evaluator_sub_shutdown,
        ]
    )
