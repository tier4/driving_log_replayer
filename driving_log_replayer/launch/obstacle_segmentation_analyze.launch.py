# Copyright (c) 2022 TierIV.inc
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

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    result_json_path_arg = DeclareLaunchArgument(
        "result_json_path", default_value="/tmp/result.jsonl", description="result json save path"
    )

    analyze_script_path = os.path.join(
        get_package_prefix("driving_log_replayer"),
        "lib",
        "driving_log_replayer",
        "obstacle_segmentation_analyzer.py",
    )

    analyzer_config_path = os.path.join(
        get_package_share_directory("driving_log_replayer_analyzer"), "config", "config.yaml"
    )

    analyze_cmd = ExecuteProcess(
        cmd=[
            "python3",
            analyze_script_path,
            LaunchConfiguration("result_json_path"),
            "-c",
            analyzer_config_path,
        ]
    )

    return launch.LaunchDescription([result_json_path_arg, analyze_cmd])
