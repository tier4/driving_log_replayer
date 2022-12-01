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
import math
from typing import Dict


@dataclass
class Position:
    x: float = None  # vehicle front
    y: float = None  # vehicle side (right side is positive)
    z: float = None

    def __init__(self, data: Dict = {}) -> None:
        self.try_parse_dict(data)

    def try_parse_dict(self, data):
        if isinstance(data, dict):
            try:
                self.y = data["x"]
                self.x = data["y"]
                self.z = data["z"]
            except KeyError:
                pass
        elif isinstance(data, list):
            try:
                self.y = data[0]
                self.x = data[1]
                self.z = data[2]
            except IndexError:
                pass
        else:
            raise NotImplementedError("Input data should be a dict or list.")

    def validate(self):
        return self.x is not None and self.y is not None and self.z is not None

    def get_xy_distance(self) -> float:
        if self.validate():
            return math.hypot(self.x, self.y)
        else:
            return None

    def add_overhang(self, val: float):
        self.y = self.y + val

    def sub_overhang(self, val: float):
        self.y = self.y - val
