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
from sys import float_info
from typing import Literal

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import KeyValue
from pydantic import BaseModel
from pydantic import Field
from pydantic import model_validator
from rosidl_runtime_py import message_to_ordereddict

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario


class StartEnd(BaseModel):
    start: float | int = Field(gt=0.0)
    end: float | int | None = Field(float_info.max, gt=0.0)

    @model_validator(mode="after")
    def validate_start_end(self) -> "StartEnd":
        err_msg = "start must be a time before end"

        if self.end < self.start:
            raise ValueError(err_msg)
        return self


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
    Hertz: float = Field(gt=0.0)
    ControlConditions: list[TimeRangeCondition] = []
    PlanningConditions: list[TimeRangeCondition] = []


class Evaluation(BaseModel):
    UseCaseName: Literal["planning_control"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Conditions: Conditions


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation


def float_stamp(stamp: Time) -> float:
    return stamp.sec + stamp.nanosec / pow(10, 9)


@dataclass
class Metrics(EvaluationItem):
    hz: float = 10.0
    rate: float = 0.9

    def set_frame(self, msg: DiagnosticArray) -> dict | None:
        self.condition: TimeRangeCondition
        now = float_stamp(msg.header.stamp)
        eval_start = self.condition.TimeRange.start
        eval_duration = now - eval_start
        eval_count = eval_duration / self.hz * self.rate
        if not (eval_start <= now <= self.condition.TimeRange.end):
            return None
        for status in msg.status:
            if status.name != self.condition.Module:
                continue
            if status.values[0].key != self.condition.Value0Key:
                continue
            self.total += 1

            frame_success = "Fail"
            if self.condition.Value0Value == status.values[0].value:
                if self.condition.DetailedConditions is None:
                    frame_success = "Success"
                    self.passed += 1
                else:
                    frame_success = "Success"
                    for key_value in status.values[1:]:
                        key_value: KeyValue
                        dc = self.condition.DetailedConditions
                        detail_field: LowerUpper = getattr(dc, key_value.key)
                        if detail_field.lower <= key_value.value <= detail_field.upper:
                            frame_success = "Fail"
                    if frame_success:
                        self.passed += 1
            self.success = self.passed >= eval_count
            return {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {"condition": self.name, "topic": message_to_ordereddict(msg)},
            }
        return None


class MetricsClassContainer:
    def __init__(self, conditions: list[TimeRangeCondition], hz: float) -> None:
        self.__container: list[Metrics] = []
        for i, time_cond in enumerate(conditions):
            self.__container.append(Metrics(f"condition{i}", time_cond, hz=hz))

    def set_frame(self, msg: DiagnosticArray) -> dict:
        frame_result: dict[int, dict] = {}
        for i, evaluation_item in enumerate(self.__container):
            result_i = evaluation_item.set_frame(msg)
            if result_i is not None:
                frame_result[i] = result_i
        return frame_result

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = [] if len(self.__container) != 0 else ["NotTested"]
        for i, evaluation_item in enumerate(self.__container):
            if not evaluation_item.success:
                rtn_success = False
                rtn_summary.append(f"condition{i} (Fail)")
            else:
                rtn_summary.append(f"condition{i} (Success)")
        prefix_str = "Passed" if rtn_success else "Failed"
        rtn_summary_str = prefix_str + ":" + ", ".join(rtn_summary)
        return (rtn_success, rtn_summary_str)


class PlanningControlResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__control_container = MetricsClassContainer(
            condition.ControlConditions,
            condition.Hertz,
        )
        self.__planning_container = MetricsClassContainer(
            condition.PlanningConditions,
            condition.Hertz,
        )

    def update(self) -> None:
        control_success, control_summary = self.__control_container.update()
        planning_success, planning_summary = self.__planning_container.update()
        self._success = control_success and planning_success
        self._summary = "Control: " + control_summary + " Planning: " + planning_summary

    def set_frame(self, msg: DiagnosticArray, module: str) -> None:
        if module == "control":
            self._frame = self.__control_container.set_frame(msg)
        if module == "planning":
            self._frame = self.__planning_container.set_frame(msg)
        self.update()
