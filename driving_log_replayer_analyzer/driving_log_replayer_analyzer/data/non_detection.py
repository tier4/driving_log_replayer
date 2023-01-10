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

from driving_log_replayer_analyzer.data.position import Position


@dataclass
class FpDistance:
    very_near: float
    near: float
    medium: float
    far: float


@dataclass
class NonDetection:
    result: str
    frame: int
    ego_position: Position
    pointcloud_points: float
    distance: Dict[int, int]

    def __init__(self, json_dict: Dict) -> None:
        self.result = ""
        self.frame = ""
        self.pointcloud_points = 0
        self.distance = {}
        self.ego_position = Position()
        try:
            self.result = json_dict["Frame"]["NonDetection"]["Result"]
            self.frame = int(json_dict["Frame"]["FrameName"])
            self.ego_position = Position(
                json_dict["Frame"]["Ego"]["TransformStamped"]["transform"]["translation"]
            )
            self.pointcloud_points = float(
                json_dict["Frame"]["NonDetection"]["Info"][0]["PointCloud"]["NumPoints"]
            )
            for distance_str, num_points in json_dict["Frame"]["NonDetection"]["Info"][0][
                "PointCloud"
            ]["Distance"].items():
                distance_list = distance_str.split(sep="-")
                dist = (int(distance_list[1]) + int(distance_list[0])) / 2
                self.distance[dist] = num_points
        except (KeyError, IndexError):
            pass

    def get_points_within_dist(self, threshold: float) -> int:
        sum_points = 0
        for dist, num_points in self.distance.items():
            if dist < threshold:
                sum_points = sum_points + num_points
        return sum_points
