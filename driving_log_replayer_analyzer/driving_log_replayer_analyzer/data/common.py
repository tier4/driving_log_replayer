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

from dataclasses import dataclass
from typing import Dict


@dataclass
class Common:
    timestamp_system: float = -1.0
    timestamp_ros: float = -1.0

    def __init__(self, json_dict: Dict) -> None:
        try:
            self.timestamp_system = json_dict["Stamp"]["System"]
            self.timestamp_ros = json_dict["Stamp"]["ROS"]
        except (KeyError, IndexError):
            pass

    def validate(self):
        return self.timestamp_system > 0.0 and self.timestamp_ros > 0.0
