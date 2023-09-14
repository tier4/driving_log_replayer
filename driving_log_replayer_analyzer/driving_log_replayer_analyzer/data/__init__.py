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
from enum import Enum
import math


class DistType(Enum):
    EUCLID = 0
    X = 1
    Y = 2


def convert_str_to_dist_type(dist_type_str: str) -> DistType:
    if dist_type_str == "front":
        return DistType.Y
    if dist_type_str == "side":
        return DistType.X
    if dist_type_str == "euclidean":
        return DistType.EUCLID
    error_msg = "Unknown distance type."
    raise RuntimeError(error_msg)  # EM101


@dataclass
class Stamp:
    timestamp_system: float = -1.0
    timestamp_ros: float = -1.0

    def __init__(self, json_dict: dict) -> None:
        try:
            self.timestamp_system = json_dict["Stamp"]["System"]
            self.timestamp_ros = json_dict["Stamp"]["ROS"]
        except (KeyError, IndexError):
            pass

    def validate(self) -> bool:
        return self.timestamp_system > 0.0 and self.timestamp_ros > 0.0  # noqa


@dataclass
class Position:
    x: float = None  # vehicle side (right side is positive)
    y: float = None  # vehicle front
    z: float = None

    def __init__(self, data: dict | None = None) -> None:
        if data is None:
            data = {}
        self.try_parse_dict(data)

    def try_parse_dict(self, data) -> None:  # noqa
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
            error_msg = "Input data should be a dict or list."
            raise NotImplementedError(error_msg)  # EM101

    def validate(self) -> bool:
        return self.x is not None and self.y is not None and self.z is not None

    def get_distance(self, dist_type: DistType) -> float:
        if self.validate():
            if dist_type == DistType.X:
                return abs(self.x)
            if dist_type == DistType.Y:
                return abs(self.y)
            return math.hypot(self.x, self.y)
        return None

    def add_overhang(self, val: float) -> None:
        self.y = self.y + val

    def sub_overhang(self, val: float) -> None:
        self.y = self.y - val
