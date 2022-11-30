import csv
import json
from pathlib import Path
import sys
from typing import List

from .config import Config
from .data.common import Common
from .data.detection import Detection
from .data.non_detection import FpDistance
from .data.non_detection import NonDetection
from .data.summary import Summary


class JsonlParser:
    summary: Summary
    common: List[Common]
    detection: List[Detection]
    non_detection: List[NonDetection]

    def __init__(self, filepath: Path, config: Config) -> None:
        self.summary = Summary()
        self.common = []
        self.detection = []
        self.non_detection = []
        self._read_jsonl_results(filepath)
        self._modify_center_from_baselink_to_overhang(config.overhang_from_baselink)
        self.summary.update_visible_range(self.get_bb_distance())

    def _read_jsonl_results(self, path: Path):
        with open(path, "r") as f:
            lines = f.read().splitlines()

        for line in lines:
            json_dict = json.loads(line)
            self.summary.update(json_dict)
            common = Common(json_dict)
            if common.validate():
                self.common.append(common)
            self.detection.append(Detection(json_dict))
            self.non_detection.append(NonDetection(json_dict))

    def _modify_center_from_baselink_to_overhang(self, overhang: float):
        for record in self.detection:
            for detection_info in record.detection_info:
                if detection_info.annotation_position.validate():
                    detection_info.annotation_position.sub_overhang(overhang)
                    detection_info.annotation_distance = (
                        detection_info.annotation_position.get_xy_distance()
                    )
                if detection_info.pointcloud_nearest_position.validate():
                    detection_info.pointcloud_nearest_position.sub_overhang(overhang)
                    detection_info.pointcloud_nearest_distance = (
                        detection_info.pointcloud_nearest_position.get_xy_distance()
                    )

        for record in self.non_detection:
            if record.ego_position.validate():
                record.ego_position.add_overhang(overhang)

    def export_to_csv(self, output_path: Path):
        """データをCSV出力する。暫定的に必要なデータのみ出力する。
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
        previous_time = self.common[0].timestamp_system
        for frame in self.common:
            time_diff = frame.timestamp_system - previous_time
            if time_diff < sys.float_info.min:
                continue
            ret.append([index, 1.0 / time_diff, ""])
            index = index + 1
            previous_time = frame.timestamp_system
        return ret

    def get_bb_position(self) -> List:
        """自車を基準としたアノテーションバウンディングボックス(BB)の中心点(x, y)とSuccess/Failのフレーム毎のリストを作成する"""

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
        """自車を基準としたアノテーションバウンディングボックス(BB)のPointCloudの最近傍点、ラベルとしてPointCloudを付与したリストを作成する"""

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
        """自車を基準としたBBの最近傍点の距離とResult

        Returns:
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

    def _split_list_per_uuid(self, input_list: List) -> List:
        # UUIDごとにリストを分割
        uuid_set = set([x[2] for x in input_list])  # UUIDの重複を削除したset
        ret = []
        for uuid in uuid_set:
            ret.append([x for x in input_list if x[2] == uuid])
        return ret

    def get_pointcloud_points_per_uuid(self) -> List:
        """Detectionで検出した自車～Annotation BB内の最近傍PCの距離ごとの検知点群数のリストを返す

        Returns:
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
        """自車～Annotation BBの最近傍点と検知点群の最近傍点との距離差

        Returns:
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
        """Non detection評価のフレームごとの点群数の累積を計算する

        Returns:
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
        """自車の位置とNon detectionのNumPointsのリストを作成する

        Returns:
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
