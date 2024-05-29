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
    ControlConditions: list[TimeRangeCondition]


class Evaluation(BaseModel):
    UseCaseName: Literal["planning_control"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Conditions: Conditions


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation


def float_stamp(stamp: Time) -> float:
    return stamp.sec + stamp.nanosec / pow(10, 9)


@dataclass
class Control(EvaluationItem):
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

            frame_success = False
            if self.condition.Value0Value == status.values[0].value:
                if self.condition.DetailedConditions is None:
                    frame_success = True
                    self.passed += 1
                else:
                    frame_success = True
                    for key_value in status.values[1:]:
                        key_value: KeyValue
                        # ここ変数でうまくやりたい
                        if key_value.key == "pos_x" and not (
                            self.condition.DetailedConditions.pos_x.lower
                            <= key_value.value
                            <= self.condition.DetailedConditions.pos_x.upper
                        ):
                            frame_success = False
                        if key_value.key == "pos_y" and not (
                            self.condition.DetailedConditions.pos_y.lower
                            <= key_value.value
                            <= self.condition.DetailedConditions.pos_y.upper
                        ):
                            frame_success = False
                        if key_value.key == "val" and not (
                            self.condition.DetailedConditions.vel.lower
                            <= key_value.value
                            <= self.condition.DetailedConditions.vel.upper
                        ):
                            frame_success = False
                    if frame_success:
                        self.passed += 1
            self.success = self.passed >= eval_count
            return {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": self.name,
            }
        return None


class ControlClassContainer:
    def __init__(self, control_conditions: list[TimeRangeCondition], hz: float) -> None:
        self.__container: list[Control] = []
        for i, time_cond in enumerate(control_conditions):
            self.__container.append(Control(f"condition{i}", time_cond, hz=hz))

    def set_frame(self, msg: DiagnosticArray) -> dict:
        frame_result: dict[int, dict] = {}
        for i, evaluation_item in enumerate(self.__container):
            result_i = evaluation_item.set_frame(msg)
            if result_i is not None:
                frame_result[i] = result_i
        return frame_result

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = []
        for i, evaluation_item in enumerate(self.__container):
            if not evaluation_item.success:
                rtn_success = False
                rtn_summary.append(f"{i} (Fail)")
            else:
                rtn_summary.append(f"{i} (Success)")
        prefix_str = "Passed" if rtn_success else "Failed"
        rtn_summary_str = prefix_str + ":".join(rtn_summary)
        return (rtn_success, rtn_summary_str)


class PlanningControlResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__control_container = ControlClassContainer(
            condition=condition.ControlConditions,
            hz=condition.Hertz,
        )
        # self.__planing_container = PlanningCOntainer(condition=condition.PlanningConditions)

    def update(self) -> None:
        self._success, self._summary = self.__control_container.update()

    def set_frame(self, msg: DiagnosticArray, module: str) -> None:
        if module == "control":
            self._frame = self.__control_container.set_frame(msg)
        # if module == "planning":
        #     self._frame = self.__planning_container.set_frame(msg)
        self.update()
