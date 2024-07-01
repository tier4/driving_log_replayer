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

from launch import LaunchDescription
from launch.actions import OpaqueFunction

import driving_log_replayer.launch_common as cmn


def generate_launch_description() -> LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=cmn.ensure_arg_compatibility),
            OpaqueFunction(function=cmn.launch_autoware),
            OpaqueFunction(function=cmn.launch_bag_player),
            OpaqueFunction(function=cmn.launch_bag_recorder),
        ],
    )
