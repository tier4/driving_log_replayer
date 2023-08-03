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

import driving_log_replayer.launch_common
import launch


def generate_launch_description():
    launch_arguments = driving_log_replayer.launch_common.get_driving_log_replayer_common_argument()
    fitter_launch = driving_log_replayer.launch_common.get_map_height_fitter(launch_service="true")
    autoware_launch = driving_log_replayer.launch_common.get_autoware_launch(
        perception="false", pose_source="yabloc"
    )
    rviz_node = driving_log_replayer.launch_common.get_rviz("localization.rviz")
    evaluator_node = driving_log_replayer.launch_common.get_evaluator_node("yabloc")
    player = driving_log_replayer.launch_common.get_player()
    recorder = driving_log_replayer.launch_common.get_recorder(
        "localization.qos.yaml",
        [
            "/clock",
            "/tf",
            "/localization/pose_estimator/pose",
            "/localization/kinematic_state",
            "/localization/util/downsample/pointcloud",
            "/localization/pose_estimator/points_aligned",
            "/diagnostics",
        ],
    )

    return launch.LaunchDescription(
        launch_arguments
        + [
            rviz_node,
            autoware_launch,
            fitter_launch,
            evaluator_node,
            recorder,
            player,
        ]
    )
