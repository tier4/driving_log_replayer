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

from driving_log_replayer_analyzer.data import Position
import pandas as pd


def fail_3_times_in_a_row(data: List) -> List:
    """対象点から近いほうの点3点から、連続して3点Failしている点をFailとする変換を行う.

    Args:
        data (list): [距離, 0 or 1, Success or Fail]

    Returns:
        list: Inputと同じ形式のlist。2項目目の0 or 1が変更される。
    """
    WINDOW = 3

    df = pd.DataFrame(data, columns=["Dist", "Val", "Result"])

    # 距離順にソート
    df.sort_values("Dist", ascending=True, inplace=True)
    df.reset_index(inplace=True, drop=True)
    df["Val"] = df["Val"].rolling(WINDOW, min_periods=1).max()

    return df.to_numpy().tolist()


def get_min_range(data: List) -> float:
    df = pd.DataFrame(data, columns=["Dist", "Val", "Result"])

    # Val == 0はFail, 最初にFailした距離を探索する
    minimum_fail_dist = df[df["Val"] == 0].min()["Dist"]
    if pd.isnull(minimum_fail_dist):
        minimum_fail_dist = sys.float_info.max

    # Passしたもののうち、最大の距離を計算する。ただし、一度でもFailするとダメなので、その条件も加える。
    return df[(df["Val"] == 1) & (df["Dist"] < minimum_fail_dist)].max()["Dist"]


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
