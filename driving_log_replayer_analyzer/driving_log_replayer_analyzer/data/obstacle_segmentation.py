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

import csv
import dataclasses
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Dict
from typing import List

import pandas as pd
import simplejson as json

from driving_log_replayer_analyzer.config.obstacle_segmentation import Config
from driving_log_replayer_analyzer.config.obstacle_segmentation import FpDistance
from driving_log_replayer_analyzer.data import DistType
from driving_log_replayer_analyzer.data import Position
from driving_log_replayer_analyzer.data import Stamp


def fail_3_times_in_a_row(data: List) -> List:
    """
    対象点から近いほうの点3点から、連続して3点Failしている点をFailとする変換を行う.

    Args:
    ----
        data (list): [距離, 0 or 1, Success or Fail]

    Returns:
    -------
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
class Frame:
    frame_name: int = -1

    def __init__(self, json_dict: Dict) -> None:
        try:
            self.frame_name = int(json_dict["Frame"]["FrameName"])
        except (KeyError, IndexError):
            pass


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
    annotation_stamp: float = None
    pointcloud_numpoints: int = None
    pointcloud_nearest_distance: float = None
    pointcloud_nearest_position: Position = Position()
    pointcloud_stamp: float = None


@dataclass
class Detection:
    result: str
    detection_info: List[DetectionInfo]

    def __init__(self, json_dict: Dict, dist_type: DistType) -> None:
        self.result = ""
        self.detection_info = []
        try:
            self.result = json_dict["Frame"]["Detection"]["Result"]
            for info in json_dict["Frame"]["Detection"]["Info"]:
                di = DetectionInfo()
                di.uuid = info["Annotation"]["UUID"]
                di.short_uuid = info["Annotation"]["UUID"][0:6]
                di.annotation_position = Position(info["Annotation"]["Position"]["position"])
                di.annotation_distance = di.annotation_position.get_distance(dist_type)
                di.pointcloud_numpoints = info["PointCloud"]["NumPoints"]
                if di.pointcloud_numpoints > 0:
                    di.pointcloud_nearest_position = Position(info["PointCloud"]["Nearest"])
                    di.pointcloud_nearest_distance = di.pointcloud_nearest_position.get_distance(
                        dist_type
                    )
                # di.annotation_stamp = info["Annotation"]["StampFloat"]
                # di.pointcloud_stamp = info["PointCloud"]["Stamp"]["sec"] + info["PointCloud"]["Stamp"]["nanosec"] * 1e-9
                self.detection_info.append(di)
        except (KeyError, IndexError):
            print("Passed frame")


@dataclass
class Summary:
    detection_pass_rate: float
    non_detection_pass_rate: float
    visible_range_one_frame: float
    visible_range_three_frame: float

    def __init__(self) -> None:
        pass

    def update_condition(self, json_dict: Dict) -> None:
        if "Condition" in json_dict.keys():
            try:
                self.detection_pass_rate = json_dict["Condition"]["Detection"]["PassRate"]
            except (KeyError, TypeError):
                self.detection_pass_rate = "N/A"
            try:
                self.non_detection_pass_rate = json_dict["Condition"]["NonDetection"]["PassRate"]
            except (KeyError, TypeError):
                self.non_detection_pass_rate = "N/A"

    def update(self, parser):
        self._update_visible_range(parser.get_bb_distance())

    def _update_visible_range(self, pass_fail_list: List):
        self.visible_range_one_frame = get_min_range(pass_fail_list)
        self.visible_range_three_frame = get_min_range(fail_3_times_in_a_row(pass_fail_list))

    def save(self, path: Path):
        with open(path.with_suffix(".json"), "w") as f:
            json.dump(dataclasses.asdict(self), f, indent=2)
            f.write("\n")


class JsonlParser:
    summary: Summary
    frame: List[Frame]
    stamp: List[Stamp]
    detection: List[Detection]
    non_detection: List[NonDetection]

    def __init__(self, filepath: Path, config: Config, dist_type: str) -> None:
        self.summary = Summary()
        self.frame = []
        self.stamp = []
        self.detection = []
        self.non_detection = []
        self._dist_type = dist_type
        self._read_jsonl_results(filepath)
        self._modify_center_from_baselink_to_overhang(config.overhang_from_baselink)

    def _read_jsonl_results(self, path: Path):
        with open(path, "r") as f:
            lines = f.read().splitlines()

        previous_dist = sys.float_info.max
        for line in lines:
            json_dict = json.loads(line)

            # tmp: 片側から車両が来るケースにおいて、自車の前（距離が最短となるとき）を通過後のデータは使わない
            try:
                position = Position(
                    json_dict["Frame"]["Detection"]["Info"][0]["Annotation"]["Position"]["position"]
                )
                if (
                    previous_dist < position.get_distance(self._dist_type)
                    and position.get_distance(self._dist_type) < 5.0
                ):
                    break
                previous_dist = position.get_distance(self._dist_type)
            except (KeyError, IndexError):
                pass

            self.summary.update_condition(json_dict)
            self.frame.append(Frame(json_dict))
            self.stamp.append(Stamp(json_dict))
            self.detection.append(Detection(json_dict, self._dist_type))
            self.non_detection.append(NonDetection(json_dict))

    def _modify_center_from_baselink_to_overhang(self, overhang: float):
        for record in self.detection:
            for detection_info in record.detection_info:
                if detection_info.annotation_position.validate():
                    detection_info.annotation_position.sub_overhang(overhang)
                    detection_info.annotation_distance = (
                        detection_info.annotation_position.get_distance(self._dist_type)
                    )
                if detection_info.pointcloud_nearest_position.validate():
                    detection_info.pointcloud_nearest_position.sub_overhang(overhang)
                    detection_info.pointcloud_nearest_distance = (
                        detection_info.pointcloud_nearest_position.get_distance(self._dist_type)
                    )

        for record in self.non_detection:
            if record.ego_position.validate():
                record.ego_position.add_overhang(overhang)

    def export_to_csv(self, output_path: Path):
        """
        データをCSV出力する。暫定的に必要なデータのみ出力する.

        TODO: detection: List[Detection]の形式を変えて、detection.py内部でリストを保持するように変更する。合わせてこの関数も移動。
        """
        with open(output_path, "w") as f:
            writer = csv.writer(f)
            # header
            writer.writerow(["UUID", "PC_Dist", "PC_NumPoints"])
            points_list = self.get_pointcloud_points_per_uuid()
            for uuid_points in points_list:
                for frame in uuid_points:
                    writer.writerow([frame[2], frame[0], frame[1]])

    def get_topic_rate(self) -> List:
        ret = []
        index = 1
        previous_time = self.stamp[0].timestamp_system
        for frame in self.stamp:
            time_diff = frame.timestamp_system - previous_time
            if time_diff < sys.float_info.min:
                continue
            ret.append([index, 1.0 / time_diff, ""])
            index = index + 1
            previous_time = frame.timestamp_system
        return ret

    def get_bb_position(self) -> List:
        """自車を基準としたアノテーションバウンディングボックス(BB)の中心点(x, y)とSuccess/Failのフレーム毎のリストを作成する."""
        ret = []
        for frame in self.detection:
            for detection_info in frame.detection_info:
                if detection_info.annotation_position.validate():
                    ret.append(
                        [
                            detection_info.annotation_position.x,
                            detection_info.annotation_position.y,
                            frame.result,
                        ]
                    )
        return ret

    def get_pointcloud_position(self) -> list:
        """自車を基準としたアノテーションバウンディングボックス(BB)のPointCloudの最近傍点、ラベルとしてPointCloudを付与したリストを作成する."""
        ret = []
        for frame in self.detection:
            for detection_info in frame.detection_info:
                if detection_info.pointcloud_nearest_position.validate():
                    ret.append(
                        [
                            detection_info.pointcloud_nearest_position.x,
                            detection_info.pointcloud_nearest_position.y,
                            "PointCloud",
                        ]
                    )
        return ret

    def get_bb_distance(self) -> List:
        """
        自車を基準としたBBの最近傍点の距離とResult.

        Returns
        -------
            list: 距離と結果を含むリスト
        """
        ret = []
        for frame in self.detection:
            for detection_info in frame.detection_info:
                if detection_info.annotation_distance is not None:
                    y_val = 1 if frame.result == "Success" else 0
                    ret.append(
                        [
                            detection_info.annotation_distance,
                            y_val,
                            frame.result,
                        ]
                    )
        return ret

    def get_bb_dist_with_stamp(self):
        ret = []
        i = 0
        for detection, frame, stamp in zip(self.detection, self.frame, self.stamp):
            for detection_info in detection.detection_info:
                if detection_info.annotation_distance is not None:
                    ret.append(
                        {
                            "x": i,
                            "y": detection_info.annotation_distance,
                            "color": detection.result,
                            "number": frame.frame_name,
                            "timestamp_ros": stamp.timestamp_ros,
                            "annotation_stamp": detection_info.annotation_stamp,
                            "pointcloud_stamp": detection_info.pointcloud_stamp,
                        }
                    )
                i = i + 1
        return ret

    def _split_list_per_uuid(self, input_list: List) -> List:
        # UUIDごとにリストを分割
        # UUIDの重複を削除したset
        uuid_set = set([x[2] for x in input_list])  # noqa: C403
        ret = []
        for uuid in uuid_set:
            ret.append([x for x in input_list if x[2] == uuid])
        return ret

    def get_pointcloud_points_per_uuid(self) -> List:
        """
        Detectionで検出した自車～Annotation BB内の最近傍PCの距離ごとの検知点群数のリストを返す.

        Returns
        -------
            list: UUIDごとの自車～Annotation BB内の最近傍PCの距離ごとの検知点群数のリスト
        """
        tmp = []
        for frame in self.detection:
            for detection_info in frame.detection_info:
                if detection_info.pointcloud_nearest_distance is not None:
                    tmp.append(
                        [
                            detection_info.pointcloud_nearest_distance,
                            detection_info.pointcloud_numpoints,
                            detection_info.short_uuid,
                        ]
                    )
        return self._split_list_per_uuid(tmp)

    def get_annotation_and_pointcloud_distance(self) -> List:
        """
        自車～Annotation BBの最近傍点と検知点群の最近傍点との距離差.

        Returns
        -------
            list: UUIDごとの自車～Annotation BBの最近傍点と検知点群の最近傍点との距離差
        """
        tmp = []
        for frame in self.detection:
            for detection_info in frame.detection_info:
                if (
                    detection_info.annotation_distance is not None
                    and detection_info.pointcloud_nearest_distance is not None
                ):
                    diff = (
                        detection_info.pointcloud_nearest_distance
                        - detection_info.annotation_distance
                    )
                    tmp.append(
                        [
                            detection_info.annotation_distance,
                            diff,
                            detection_info.short_uuid,
                        ]
                    )
        return self._split_list_per_uuid(tmp)

    def get_non_detection_frame_points(self, fp_dist: FpDistance) -> List:
        """
        Non detection評価のフレームごとの点群数の累積を計算する.

        Returns
        -------
            list: フレームと点群数の累積数、ポップアップ表示用に使用する距離毎のFP数
        """
        ret = []
        for frame in self.non_detection:
            ret.append(
                {
                    "x": frame.frame,
                    "y": frame.pointcloud_points,
                    "color": frame.result,
                    "超近傍": frame.get_points_within_dist(fp_dist.very_near),
                    "近傍": frame.get_points_within_dist(fp_dist.near),
                    "中距離": frame.get_points_within_dist(fp_dist.medium),
                    "遠距離": frame.get_points_within_dist(fp_dist.far),
                }
            )
        return ret

    def get_non_detection_position(self, fp_dist: FpDistance) -> List:
        """
        自車の位置とNon detectionのNumPointsのリストを作成する.

        Returns
        -------
            list: 自車の位置(x,y)とNon detectionのNumPoints、ポップアップ表示用に使用する距離毎のFP数
        """
        ret = []
        for frame in self.non_detection:
            if frame.result != "":
                ret.append(
                    {
                        "x": frame.ego_position.x,
                        "y": frame.ego_position.y,
                        "color": frame.pointcloud_points,
                        "超近傍": frame.get_points_within_dist(fp_dist.very_near),
                        "近傍": frame.get_points_within_dist(fp_dist.near),
                        "中距離": frame.get_points_within_dist(fp_dist.medium),
                        "遠距離": frame.get_points_within_dist(fp_dist.far),
                    }
                )
        return ret
