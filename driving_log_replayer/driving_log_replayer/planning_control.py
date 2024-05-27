# Copyright (c) 2024 TIER IV.inc
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
from pathlib import Path
from sys import float_info
from typing import Any
from typing import Literal

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from pydantic import BaseModel
from pydantic import Field
from pydantic import field_validator
import simplejson as json

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario


class StartEnd(BaseModel):
    start: float | int = Field(gt=0.0)
    end: float | int | None = Field(float_info.max, gt=0.0)

    @field_validator("end")
    @classmethod
    def validate_distance_range(cls, v: float | int, values: Any) -> float:  # noqa
        err_msg = f"{v} is not valid distance range, expected ordering start end with start < end."

        if v >= values["start"]:
            raise ValueError(err_msg)
        return float(v)


class LowerUpper(BaseModel):
    lower: float
    upper: float


class ValuesAfter0(BaseModel):
    pos_x: LowerUpper
    pos_y: LowerUpper
    vel: LowerUpper | None = None


class TimeRangeCondition(BaseModel):
    TimeRange: StartEnd
    Module: str
    Value0Key: Literal["decision"]
    Value0Value: Literal["none", "slow_down", "stop"]
    DetailedConditions: ValuesAfter0 | None = None


class Conditions(BaseModel):
    ControlConditions: list[TimeRangeCondition]


class Evaluation(BaseModel):
    UseCaseName: Literal["planning_control"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Hertz: float = Field(gt=0.0)
    Conditions: Conditions


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation


def float_stamp(stamp: Time) -> float:
    return stamp.sec + stamp.nanosecd / pow(10, 9)


@dataclass
class Control(EvaluationItem):
    hz: float

    def set_frame(self, msg: DiagnosticArray, hz: float) -> dict | None:
        self.condition: TimeRangeCondition
        if not (
            self.condition.TimeRange.start
            <= float_stamp(msg.header.stamp)
            <= self.condition.TimeRange.end
        ):
            return None
        for status in msg.status:
            if status.name != self.condition.Module:
                continue
            if status.values[0].key != self.condition.Value0Key:
                continue
            self.total += 1
            frame_success = True
            info_dict = {}
            if self.condition.Value0Value != status.values[0].value:
                frame_success = False
                info_dict[status_name] = values
                self.metrics_dict[status_name], diag_success = self.calc_average_and_success(
                    status_name,
                    values,
                )
                if diag_success is False:
                    frame_fail_items += f", {status_name}"
                frame_success = frame_success and diag_success
            self.success = frame_success
            if self.condition.Threshold == {}:
                self.summary = f"{self.name} (NotTested)"
            else:
                self.summary = f"{self.name} ({self.success_str()}{frame_fail_items})"
            return {
                "Result": {"Total": self.success_str(), "Frame": self.success_str()},
                "Info": info_dict,
                "Metrics": self.metrics_dict,
            }
        return None


class PlanningControlResult(ResultBase):
    def __init__(self, condition: Conditions, hz: float) -> None:
        super().__init__()
        self.__control_container = ControlContainer(condition=condition.ControlConditions, hz=hz)
        # self.__planing_container = PlanningCOntainer(condition=condition.PlanningConditions)

    def update(self) -> None:
        self._success, self._summary = self.__control_container.update()

    def set_frame(self, msg: DiagnosticArray, module: str) -> None:
        if module == "control":
            self._frame = self.__control_container.set_frame(msg)
        # if module == "planning":
        #     self._frame = self.__planning_container.set_frame(msg)
        self.update()
