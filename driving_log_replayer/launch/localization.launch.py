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

import launch

import driving_log_replayer.launch_common as cmn

RECORD_TOPIC_REGEX = """^/clock$\
|^/tf$\
|^/diagnostics$\
|^/localization/pose_estimator/transform_probability$\
|^/localization/pose_estimator/nearest_voxel_transformation_likelihood$\
|^/localization/pose_estimator/pose$\
|^/localization/pose_estimator/initial_to_result_relative_pose$\
|^/localization/kinematic_state$\
|^/localization/util/downsample/pointcloud$\
|^/localization/pose_estimator/points_aligned$\
|^/driving_log_replayer/.*\
"""


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    fitter_launch = cmn.get_map_height_fitter(launch_service="true")
    autoware_launch = cmn.get_autoware_launch(
        perception="false",
        pose_source="ndt",
        twist_source="gyro_odom",
    )
    rviz_node = cmn.get_rviz("localization.rviz")
    evaluator_node = cmn.get_evaluator_node("localization")
    player = cmn.get_player()
    recorder, recorder_override = cmn.get_regex_recorders(
        "localization.qos.yaml",
        RECORD_TOPIC_REGEX,
    )
    topic_monitor = cmn.get_topic_state_monitor_launch(
        "localization_topics.yaml",
    )

    return launch.LaunchDescription(
        [
            *launch_arguments,
            rviz_node,
            autoware_launch,
            fitter_launch,
            evaluator_node,
            player,
            recorder,
            recorder_override,
            topic_monitor,
        ],
    )
