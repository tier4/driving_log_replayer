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

import dataclasses
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict
from typing import List

from driving_log_replayer_analyzer.calc import fail_3_times_in_a_row
from driving_log_replayer_analyzer.calc import get_min_range


@dataclass
class Summary:
    detection_pass_rate: float
    non_detection_pass_rate: float
    visible_range_one_frame: float
    visible_range_three_frame: float

    def __init__(self) -> None:
        pass

    def update(self, json_dict: Dict) -> None:
        if "Condition" in json_dict.keys():
            try:
                self.detection_pass_rate = json_dict["Condition"]["Detection"]["PassRate"]
            except (KeyError, TypeError):
                self.detection_pass_rate = "N/A"
            try:
                self.non_detection_pass_rate = json_dict["Condition"]["NonDetection"]["PassRate"]
            except (KeyError, TypeError):
                self.non_detection_pass_rate = "N/A"

    def update_visible_range(self, pass_fail_list: List):
        self.visible_range_one_frame = get_min_range(pass_fail_list)
        self.visible_range_three_frame = get_min_range(fail_3_times_in_a_row(pass_fail_list))

    def save(self, path: Path):
        with open(path.with_suffix(".json"), "w") as f:
            json.dump(dataclasses.asdict(self), f, indent=2)
            f.write("\n")
