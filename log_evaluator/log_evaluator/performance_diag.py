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
from typing import ClassVar
from typing import Literal

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from example_interfaces.msg import Byte
from example_interfaces.msg import Float64
from pydantic import BaseModel

from log_evaluator.result import EvaluationItem
from log_evaluator.result import ResultBase
from log_evaluator.scenario import InitialPose
from log_evaluator.scenario import Scenario

INVALID_FLOAT_VALUE = -99.9


def get_diag_value(diag_status: DiagnosticStatus, key_name: str) -> str:
    key_value: KeyValue
    for key_value in diag_status.values:
        if key_value.key == key_name:
            return key_value.value
    return ""


def convert_str_to_float(str_float: str) -> float:
    try:
        return float(str_float)
    except ValueError:
        return INVALID_FLOAT_VALUE


def convert_str_to_int(str_float: str) -> float:
    try:
        return int(str_float)
    except ValueError:
        return 0


class VisibilityCondition(BaseModel):
    ScenarioType: Literal["TP", "FP"] | None
    PassFrameCount: int


class BlockageCondition(BaseModel):
    ScenarioType: Literal["TP", "FP"] | None
    PassFrameCount: int
    BlockageType: Literal["both", "ground", "sky"]


class LiDARCondition(BaseModel):
    Visibility: VisibilityCondition
    Blockage: dict[str, BlockageCondition]


class Conditions(BaseModel):
    LiDAR: LiDARCondition


class Evaluation(BaseModel):
    UseCaseName: Literal["performance_diag"]
    UseCaseFormatVersion: Literal["1.0.0"]
    Conditions: Conditions
    InitialPose: InitialPose | None


class PerformanceDiagScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Visibility(EvaluationItem):
    name: str = "Visibility"
    VALID_VALUE_THRESHOLD: ClassVar[float] = 0.0

    def __post_init__(self) -> None:
        self.condition: VisibilityCondition
        self.scenario_type: str | None = self.condition.ScenarioType
        self.valid: bool = self.scenario_type is not None

    def set_frame(self, diag_status: DiagnosticStatus) -> tuple[dict, Float64 | None, Byte | None]:
        if not self.valid:
            self.success = True
            self.summary = "Invalid"
            return (
                {"Result": {"Total": self.success_str(), "Frame": "Invalid"}, "Info": {}},
                None,
                None,
            )
        frame_success = "Fail"
        self.total += 1
        visibility_value = get_diag_value(diag_status, "value")
        diag_level = diag_status.level
        if self.scenario_type == "TP":
            if diag_level == DiagnosticStatus.ERROR:
                frame_success = "Success"
                self.passed += 1
            self.success = self.passed >= self.condition.PassFrameCount
        elif self.scenario_type == "FP":
            if diag_level != DiagnosticStatus.ERROR:
                frame_success = "Success"
                self.passed += 1
            self.success = self.passed == self.total
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total}"
        float_value = convert_str_to_float(visibility_value)
        valid_value = float_value >= Visibility.VALID_VALUE_THRESHOLD
        return (
            {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {
                    "Level": int.from_bytes(diag_level, byteorder="little"),
                    "Value": float_value,
                },
            },
            Float64(data=float_value) if valid_value else None,
            Byte(data=diag_level) if valid_value else None,
        )


@dataclass
class Blockage(EvaluationItem):
    VALID_VALUE_THRESHOLD: ClassVar[float] = 0.0

    def __post_init__(self) -> None:
        self.condition: BlockageCondition
        self.scenario_type: str | None = self.condition.ScenarioType
        self.blockage_type: str = self.condition.BlockageType
        self.pass_frame_count: int = self.condition.PassFrameCount
        self.valid: bool = self.scenario_type is not None

    def set_frame(
        self,
        diag_status: DiagnosticStatus,
    ) -> tuple[dict, Float64 | None, Float64 | None, Byte | None]:
        if not self.valid:
            self.success = True
            self.summary = "Invalid"
            return (
                {
                    self.name: {
                        "Result": {"Total": self.success_str(), "Frame": "Invalid"},
                        "Info": {},
                    },
                },
                None,
                None,
                None,
            )
        frame_success = "Fail"
        self.total += 1
        ground_ratio = get_diag_value(diag_status, "ground_blockage_ratio")
        sky_ratio = get_diag_value(diag_status, "sky_blockage_ratio")
        diag_level = diag_status.level
        if self.scenario_type == "TP":
            if diag_level == DiagnosticStatus.ERROR and self.blockage_type in diag_status.message:
                frame_success = "Success"
                self.passed += 1
            self.success = self.passed >= self.pass_frame_count
        elif self.scenario_type == "FP":
            if not (
                diag_level == DiagnosticStatus.ERROR and self.blockage_type in diag_status.message
            ):
                frame_success = "Success"
                self.passed += 1
            self.success = self.passed == self.total
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total}"
        float_sky_ratio = convert_str_to_float(sky_ratio)
        float_ground_ratio = convert_str_to_float(ground_ratio)
        valid_ratio = (
            float_sky_ratio >= Blockage.VALID_VALUE_THRESHOLD
            and float_ground_ratio >= Blockage.VALID_VALUE_THRESHOLD
        )
        return (
            {
                self.name: {
                    "Result": {
                        "Total": self.success_str(),
                        "Frame": frame_success,
                    },
                    "Info": {
                        "Level": int.from_bytes(diag_level, byteorder="little"),
                        "GroundBlockageRatio": float_ground_ratio,
                        "GroundBlockageCount": convert_str_to_int(
                            get_diag_value(diag_status, "ground_blockage_count"),
                        ),
                        "SkyBlockageRatio": float_sky_ratio,
                        "SkyBlockageCount": convert_str_to_int(
                            get_diag_value(diag_status, "sky_blockage_count"),
                        ),
                    },
                },
            },
            Float64(data=float_sky_ratio) if valid_ratio else None,
            Float64(data=float_ground_ratio) if valid_ratio else None,
            Byte(data=diag_level) if valid_ratio else None,
        )


class PerformanceDiagResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__visibility = Visibility(condition=condition.LiDAR.Visibility)
        self.__blockages: dict[str, Blockage] = {}
        for lidar_name, v in condition.LiDAR.Blockage.items():
            if v.ScenarioType is not None:
                self.__blockages[lidar_name] = Blockage(name=lidar_name, condition=v)

    def update(self) -> None:
        tmp_success = self.__visibility.success
        tmp_summary = self.__visibility.summary + " Blockage:"
        for v in self.__blockages.values():
            tmp_summary += " " + v.summary
            if not v.success:
                tmp_success = False
        prefix_str = "Passed: " if tmp_success else "Failed: "
        self._success = tmp_success
        self._summary = prefix_str + tmp_summary

    def set_frame(self) -> None:
        # abstract method
        pass

    def set_visibility_frame(
        self,
        msg: DiagnosticStatus,
        map_to_baselink: dict,
    ) -> None:
        out_frame = {"Ego": {"TransformStamped": map_to_baselink}}
        out_frame["Visibility"], msg_visibility_value, msg_visibility_level = (
            self.__visibility.set_frame(msg)
        )
        self._frame = out_frame
        self.update()
        return msg_visibility_value, msg_visibility_level

    def set_blockage_frame(
        self,
        msg: DiagnosticStatus,
        map_to_baselink: dict,
        lidar_name: str,
    ) -> None:
        out_frame = {"Ego": {"TransformStamped": map_to_baselink}}
        (
            out_frame["Blockage"],
            msg_blockage_sky_ratio,
            msg_blockage_ground_ratio,
            msg_blockage_level,
        ) = self.__blockages[lidar_name].set_frame(msg)
        self._frame = out_frame
        self.update()
        return msg_blockage_sky_ratio, msg_blockage_ground_ratio, msg_blockage_level
