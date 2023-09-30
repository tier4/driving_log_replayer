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


from dataclasses import dataclass
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

INVALID_FLOAT_VALUE = -99.9


def get_diag_value(diag_status: DiagnosticStatus, key_name: str) -> str:
    key_value: KeyValue
    for key_value in diag_status.values:
        if key_value.key == key_name:
            return key_value.value
    return ""


def parse_str_float(str_float: str) -> float:
    try:
        return float(str_float)
    except ValueError:
        return INVALID_FLOAT_VALUE


@dataclass
class Visibility(EvaluationItem):
    name: ClassVar[str] = "Visibility"
    success = True
    REGEX_VISIBILITY_DIAG_NAME: ClassVar[
        str
    ] = "/autoware/sensing/lidar/performance_monitoring/visibility/.*"
    VALID_VALUE_THRESHOLD: ClassVar[float] = 0.0

    def __post_init__(self) -> None:
        self.scenario_type: str | None = self.condition.get("ScenarioType")
        self.valid: bool = self.scenario_type is not None

    def set_frame(self, msg: DiagnosticArray) -> tuple[dict, Float64 | None, Byte | None]:
        if not self.valid:
            return (
                {"Result": {"Total": self.success_str(), "Frame": "Invalid"}, "Info": {}},
                None,
                None,
            )
        include_target_status = False
        frame_success = "Fail"
        self.total += 1
        diag_status: DiagnosticStatus
        for diag_status in msg.status:
            if not re.fullmatch(Visibility.REGEX_VISIBILITY_DIAG_NAME, diag_status.name):
                continue
            include_target_status = True
            visibility_value = get_diag_value(diag_status, "value")
            diag_level = diag_status.level
            if self.scenario_type == "TP":
                if diag_level == DiagnosticStatus.ERROR:
                    frame_success = "Success"
                    self.passed += 1
                self.success = self.passed >= self.condition["PassFrameCount"]
            elif self.scenario_type == "FP":
                if diag_level != DiagnosticStatus.ERROR:
                    frame_success = "Success"
                    self.passed += 1
                self.success = self.passed == self.total
            self.summary = f"{self.name} ({self.success_str()}: {self.passed} / {self.total})"
            break
        if include_target_status:
            float_value = parse_str_float(visibility_value)
            valid_value = float_value >= Visibility.VALID_VALUE_THRESHOLD
            return (
                {
                    "Result": {"Total": self.success_str(), "Frame": frame_success},
                    "Info": {
                        "Level": int.from_bytes(diag_level, byteorder="little"),
                        "Value": visibility_value,
                    },
                },
                Float64(data=float(visibility_value)) if valid_value else None,
                Byte(data=diag_level) if valid_value else None,
            )
        return (
            {
                "Result": {"Total": self.success_str(), "Frame": "Warn"},
                "Info": {"Reason": "diagnostics does not contain visibility"},
            },
            None,
            None,
        )


@dataclass
class Blockage(EvaluationItem):
    success: bool = True
    passed_sensors: dict[str, int] = field(default_factory=dict)
    total_sensors: dict[str, int] = field(default_factory=dict)
    success_sensors: dict[str, bool] = field(default_factory=dict)
    BLOCKAGE_DIAG_BASE_NAME: ClassVar[
        str
    ] = "/autoware/sensing/lidar/performance_monitoring/blockage/blockage_return_diag:  sensing lidar"
    REGEX_BLOCKAGE_DIAG_NAME: ClassVar[str] = BLOCKAGE_DIAG_BASE_NAME + ".*"
    VALID_VALUE_THRESHOLD: ClassVar[float] = 0.0

    def __post_init__(self) -> None:
        self.valid = False
        for k, v in self.condition.items():
            if v.get("ScenarioType") is not None:
                self.valid = True
                self.passed_sensors[k] = 0
                self.total_sensors[k] = 0
                self.success_sensors[k] = True

    def lidar_success_str(self, lidar_name: str) -> str:
        return "Success" if self.success_sensor[lidar_name] else "Fail"

    @classmethod
    def trim_lidar_name(cls, diag_name: str) -> str:
        remove_prefix = diag_name.replace(f"{Blockage.BLOCKAGE_DIAG_BASE_NAME} ", "")
        return remove_prefix.replace(": blockage_validation", "")

    def set_frame(self, msg: DiagnosticStatus) -> tuple[dict, dict, dict, dict]:
        if not self.valid:
            return (
                {"Result": {"Total": self.success_str(), "Frame": "Invalid"}, "Info": {}},
                {},
                {},
                {},
            )
        include_target_status = False
        frame_success = "Fail"
        self.total += 1
        rtn_dict = {}
        rtn_sky_ratio = {}
        rtn_ground_ratio = {}
        rtn_level = {}
        diag_status: DiagnosticStatus
        for diag_status in msg.status:
            if not re.fullmatch(Blockage.REGEX_BLOCKAGE_DIAG_NAME, diag_status.name):
                continue
            lidar_name = Blockage.trim_lidar_name(diag_status.name)
            scenario_type = self.condition[lidar_name]["ScenarioType"]
            if scenario_type is None:
                continue
            include_target_status = True
            frame_success = "Fail"
            self.total_sensors[lidar_name] += 1
            ground_ratio = get_diag_value(diag_status, "ground_blockage_ratio")
            sky_ratio = get_diag_value(diag_status, "sky_blockage_ratio")
            diag_level = diag_status.level
            if scenario_type == "TP":
                if (
                    diag_level == DiagnosticStatus.ERROR
                    and self.condition[lidar_name]["BlockageType"] in diag_status.message
                ):
                    frame_success = "Success"
                    self.passed_sensors[lidar_name] += 1
                self.success_sensors[lidar_name] = (
                    self.passed_sensors[lidar_name] >= self.condition[lidar_name]["PassFrameCount"]
                )
            elif scenario_type == "FP":
                if diag_level != DiagnosticStatus.ERROR:
                    frame_success = "Success"
                    self.passed_sensors[lidar_name] += 1
                self.success_sensors[lidar_name] = (
                    self.passed_sensors[lidar_name] != self.total_sensors[lidar_name]
                )
            rtn_dict[lidar_name] = {
                "Result": {"Total": self.lidar_success_sensors(lidar_name), "Frame": frame_success},
                "Info": {
                    "Level": int.from_bytes(
                        diag_level,
                        int.from_bytes(diag_level, byteorder="little"),
                    ),
                    "GroundBlockageRatio": ground_ratio,
                    "GroundBlockageCount": get_diag_value(diag_status, "ground_blockage_count"),
                    "SkyBlockageRatio": sky_ratio,
                    "SkyBlockageCount": get_diag_value(diag_status, "sky_blockage_count"),
                },
            }
            float_sky_ratio = parse_str_float(sky_ratio)
            float_ground_ratio = parse_str_float(ground_ratio)
            valid_ratio = (
                float_sky_ratio >= Blockage.VALID_VALUE_THRESHOLD
                and float_ground_ratio >= Blockage.VALID_VALUE_THRESHOLD
            )
            rtn_sky_ratio[lidar_name] = Float64(data=float_sky_ratio) if valid_ratio else None
            rtn_ground_ratio[lidar_name] = Float64(data=float_ground_ratio) if valid_ratio else None
            rtn_level[lidar_name] = Byte(data=diag_level) if valid_ratio else None
        self.update()
        if include_target_status:
            return rtn_dict, rtn_sky_ratio, rtn_ground_ratio, rtn_level
        return (
            {
                "Result": {"Total": self.success_str(), "Frame": "Warn"},
                "Info": {"Reason": "diagnostics does not contain blockage"},
            },
            {},
            {},
            {},
        )

    def update(self) -> None:
        tmp_success = True
        tmp_summary = ""
        for lidar_name, v in self.success_sensors:
            tmp_summary += f"{lidar_name}: {self.passed_sensors[lidar_name]} / {self.total_sensors[lidar_name]} "
            # 1個でもFalseが入ってたらlidar試験全体がfalse
            if not v:
                tmp_success = False
        self.success = tmp_success
        self.summary = tmp_summary


class PerformanceDiagResult(ResultBase):
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__visibility = Visibility(condition=condition.get("LiDAR", {}).get("Visibility", {}))
        self.__blockage = Blockage(condition=condition.get("LiDAR", {}).get("Blockage", {}))

    def update(self) -> None:
        summary_str = f"{self.__visibility.summary}, {self.__blockage.summary}"
        if self.__visibility.success and self.__blockage.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = True
            self._summary = f"Failed: {summary_str}"

    def set_frame(
        self,
        msg: DiagnosticArray,
        map_to_baselink: dict,
    ) -> tuple[
        Float64 | None,
        Byte | None,
        dict[str, Float64],
        dict[str, Float64],
        dict[str, Byte],
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
