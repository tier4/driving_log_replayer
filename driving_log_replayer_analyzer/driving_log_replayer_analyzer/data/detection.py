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
from typing import List

from .position import Position


@dataclass
class DetectionInfo:
    uuid: str = None
    short_uuid: str = None
    annotation_position: Position = Position()
    annotation_distance: float = None
    pointcloud_numpoints: int = None
    pointcloud_nearest_distance: float = None
    pointcloud_nearest_position: Position = Position()


@dataclass
class Detection:
    result: str
    detection_info: List[DetectionInfo]

    def __init__(self, json_dict: Dict) -> None:
        self.result = ""
        self.detection_info = []
        try:
            self.result = json_dict["Frame"]["Detection"]["Result"]
            for info in json_dict["Frame"]["Detection"]["Info"]:
                di = DetectionInfo()
                di.uuid = info["Annotation"]["UUID"]
                di.short_uuid = info["Annotation"]["UUID"][0:6]
                di.annotation_position = Position(info["Annotation"]["Position"]["position"])
                di.annotation_distance = di.annotation_position.get_xy_distance()
                di.pointcloud_numpoints = info["PointCloud"]["NumPoints"]
                if di.pointcloud_numpoints > 0:
                    di.pointcloud_nearest_position = Position(info["PointCloud"]["Nearest"])
                    di.pointcloud_nearest_distance = (
                        di.pointcloud_nearest_position.get_xy_distance()
                    )
                self.detection_info.append(di)
        except (KeyError, IndexError):
            pass
