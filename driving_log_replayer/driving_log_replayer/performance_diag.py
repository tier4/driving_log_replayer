#!/usr/bin/env python3

# Copyright (c) 2022 TIER IV.inc
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


from dataclasses import field
import re
from typing import ClassVar

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from example_interfaces.msg import Byte
from example_interfaces.msg import Float64

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase


def get_diag_value(diag_status: DiagnosticStatus, key_name: str) -> str:
    key_value: KeyValue
    for key_value in diag_status.values:
        if key_value.key == key_name:
            return key_value.value
    return ""


class Visibility(EvaluationItem):
    success = True
    REGEX_VISIBILITY_DIAG_NAME: ClassVar[
        str
    ] = "/autoware/sensing/lidar/performance_monitoring/visibility/.*"

    def set_frame(self, msg: DiagnosticArray) -> tuple[dict, Float64 | None, Byte | None]:
        for diag_status in msg.status:
            if re.fullmatch(Visibility.REGEX_VISIBILITY_DIAG_NAME, diag_status.name):
                (
                    visibility_result,
                    msg_visibility_value,
                    msg_visibility_level,
                ) = self.summarize_visibility(diag_status)
                visibility_results.append(visibility_result)

    def summarize_visibility(
        self,
        diag_status: DiagnosticStatus,
    ) -> tuple[dict, Float64 | None, Byte | None]:
        result = "Skipped"
        info = []
        scenario_type = self.__visibility_condition["ScenarioType"]
        msg_value = None
        msg_level = None
        if scenario_type is not None:
            self.__visibility_result = True
            visibility_value = get_diag_value(diag_status, "value")
            diag_level = diag_status.level
            if scenario_type == "TP":
                if diag_level == DiagnosticStatus.ERROR:
                    result = "Success"
                    self.__visibility_success += 1
                else:
                    result = "Fail"
                # visibilityは1個なので、TPかTPかで更新したら、全体結果についての判定も出来る
                if self.__visibility_success < self.__visibility_condition["PassFrameCount"]:
                    self.__visibility_result = False
            elif scenario_type == "FP":
                if diag_level == DiagnosticStatus.ERROR:
                    result = "Fail"
                else:
                    result = "Success"
                    self.__visibility_success += 1
                # visibilityは1個なので、TPかTPかで更新したら、全体結果についての判定も出来る
                if self.__visibility_success != self.__visibility_total:
                    self.__visibility_result = False
            info.append(
                {
                    "Level": int.from_bytes(diag_level, byteorder="little"),
                    "Visibility": visibility_value,
                },
            )
            self.__visibility_msg = (
                f"passed: {self.__visibility_success} / {self.__visibility_total}"
            )
            # publishするmsgを作る
            float_value = float(visibility_value)
            if float_value >= PerformanceDiagResult.VALID_VALUE_THRESHOLD:
                msg_value = Float64()
                msg_value.data = float_value
                msg_level = Byte()
                msg_level.data = diag_level
        return {"Result": result, "Info": info}, msg_value, msg_level


class Blockage(EvaluationItem):
    success = True
    passed_sensors = field(default_factory=dict)
    success_sensors = field(default_factory=dict)
    BLOCKAGE_DIAG_BASE_NAME: ClassVar[
        str
    ] = "/autoware/sensing/lidar/performance_monitoring/blockage/blockage_return_diag:  sensing lidar"
    REGEX_BLOCKAGE_DIAG_NAME: ClassVar[str] = BLOCKAGE_DIAG_BASE_NAME + ".*"

    def __post_init__(self) -> None:
        for k, v in self.condition.items():
            if v.get("ScenarioType") is not None:
                self.passed_sensors[k] = 0
                self.success_sensors[k] = True

    @classmethod
    def trim_lidar_name(cls, diag_name: str) -> str:
        remove_prefix = diag_name.replace(f"{Blockage.BLOCKAGE_DIAG_BASE_NAME} ", "")
        return remove_prefix.replace(": blockage_validation", "")

    def summarize_blockage(
        self,
        diag_status: DiagnosticStatus,
    ) -> tuple[dict, Float64 | None, Float64 | None, Byte | None]:
        result = "Skipped"
        info = []
        lidar_name = Blockage.trim_lidar_name(diag_status.name)
        info_dict = {"Name": lidar_name}
        scenario_type = self.__blockage_condition[lidar_name]["ScenarioType"]
        msg_sky_ratio = None
        msg_ground_ratio = None
        msg_level = None
        if scenario_type is not None:
            self.__blockage_lidar_result[lidar_name] = True
            ground_ratio = get_diag_value(diag_status, "ground_blockage_ratio")
            sky_ratio = get_diag_value(diag_status, "sky_blockage_ratio")
            diag_level = diag_status.level
            if scenario_type == "TP":
                if diag_level == DiagnosticStatus.ERROR:
                    # sky, ground, bothが一致しているか確認する
                    blockage_type = self.__blockage_condition[lidar_name]["BlockageType"]
                    if blockage_type in diag_status.message:
                        result = "Success"
                        self.__blockage_lidar_success[lidar_name] += 1
                    else:
                        result = "Fail"
                else:
                    result = "Fail"
                # blockageはlidarの数分ある。個別のlidarの結果の更新を行う
                if (
                    self.__blockage_lidar_success[lidar_name]
                    < self.__blockage_condition[lidar_name]["PassFrameCount"]
                ):
                    self.__blockage_lidar_result[lidar_name] = False
            elif scenario_type == "FP":
                if diag_level == DiagnosticStatus.ERROR:
                    result = "Fail"
                else:
                    result = "Success"
                    self.__blockage_lidar_success[lidar_name] += 1
                # blockageはlidarの数分ある。個別のlidarの結果の更新を行う
                if self.__blockage_lidar_success[lidar_name] != self.__blockage_total:
                    self.__blockage_lidar_result[lidar_name] = False
            info_dict["Level"] = int.from_bytes(diag_level, byteorder="little")
            info_dict["GroundBlockageRatio"] = ground_ratio
            info_dict["GroundBlockageCount"] = get_diag_value(diag_status, "ground_blockage_count")
            info_dict["SkyBlockageRatio"] = sky_ratio
            info_dict["SkyBlockageCount"] = get_diag_value(diag_status, "sky_blockage_count")
            # publishするmsgを作る
            float_sky_ratio = float(sky_ratio)
            float_ground_ratio = float(ground_ratio)
            if (
                float_sky_ratio >= PerformanceDiagResult.VALID_VALUE_THRESHOLD
                and float_ground_ratio >= PerformanceDiagResult.VALID_VALUE_THRESHOLD
            ):
                msg_sky_ratio = Float64()
                msg_sky_ratio.data = float_sky_ratio
                msg_ground_ratio = Float64()
                msg_ground_ratio.data = float_ground_ratio
                msg_level = Byte()
                msg_level.data = diag_level
        info.append(info_dict)
        return {"Result": result, "Info": info}, msg_sky_ratio, msg_ground_ratio, msg_level

    def update_blockage(self) -> None:
        self.__blockage_result = True
        self.__blockage_msg = ""
        for lidar_name, v in self.__blockage_lidar_result.items():
            self.__blockage_msg += f"{lidar_name}: {self.__blockage_lidar_success[lidar_name]} / {self.__blockage_total} "
            # 1個でもFalseが入ってたらlidar試験全体がfalse
            if not v:
                self.__blockage_result = False


class PerformanceDiagResult(ResultBase):
    VALID_VALUE_THRESHOLD = 0.0

    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__visibility = Visibility(condition=condition.get("LiDAR", {}).get("Visibility"))
        self.__blockage = Blockage(condition=condition.get("LiDAR", {}).get("Blockage"))

    def update(self) -> None:
        summary_str = f"{self.__visibility.summary}, {self.__blockage.summary}"
        if self.__visibility.success and self.__blockage.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = True
            self._summary = f"Failed: {summary_str}"

    def summarize_diag_agg(
        self,
        msg: DiagnosticArray,
    ) -> tuple[
        list[dict],
        Float64 | None,
        Byte | None,
        list[dict],
        dict[str, Float64 | None],
        dict[str, Float64 | None],
    ]:
        self.__visibility_total += 1
        self.__blockage_total += 1
        for diag_status in msg.status:
            if re.fullmatch(REGEX_VISIBILITY_DIAG_NAME, diag_status.name):
                (
                    visibility_result,
                    msg_visibility_value,
                    msg_visibility_level,
                ) = self.summarize_visibility(diag_status)
                visibility_results.append(visibility_result)
            elif re.fullmatch(REGEX_BLOCKAGE_DIAG_NAME, diag_status.name):
                (
                    blockage_result,
                    msg_blockage_sky_ratio,
                    msg_blockage_ground_ratio,
                    msg_blockage_level,
                ) = self.summarize_blockage(diag_status)
                blockage_results.append(blockage_result)
                lidar_name = trim_lidar_name(diag_status.name)
                msg_blockage_sky_ratios[lidar_name] = msg_blockage_sky_ratio
                msg_blockage_ground_ratios[lidar_name] = msg_blockage_ground_ratio
                msg_blockage_levels[lidar_name] = msg_blockage_level

    def set_frame(
        self,
        msg: DiagnosticArray,
        map_to_baselink: dict,
    ) -> tuple[
        Float64 | None,
        Byte | None,
        dict[str, Float64 | None],
        dict[str, Float64 | None],
        dict[str, Byte | None],
    ]:
        out_frame = {"Ego": {"TransformStamped": map_to_baselink}}
        (
            out_frame["Visibility"],
            msg_visibility_value,
            msg_visibility_level,
        ) = self.__visibility.set_frame(msg)
        (
            out_frame["Blockage"],
            msg_blockage_sky_ratios,
            msg_blockage_ground_ratios,
            msg_blockage_levels,
        ) = self.__blockage.set_frame(msg)
        self._frame = out_frame
        self.update()
        return (
            msg_visibility_value,
            msg_visibility_level,
            msg_blockage_sky_ratios,
            msg_blockage_ground_ratios,
            msg_blockage_levels,
        )
