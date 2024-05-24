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
from typing import Any
from typing import Literal

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from pydantic import BaseModel
from pydantic import Field
from pydantic import field_validator
import simplejson as json

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario

TimeRange = str
ModuleName = str
Value0Key = str
Value0Value = str
AdditionalCondition = dict


class Conditions(BaseModel):
    TimeRanges: dict[
        TimeRange,
        dict[ModuleName, dict[Value0Key, dict[Value0Value, AdditionalCondition]]],
    ]


class Evaluation(BaseModel):
    UseCaseName: Literal["planning_control"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Hertz: float = Field(gt=0.0)
    Conditions: Conditions


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation


class PlanningControlResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__deviation = DeviationClassContainer(condition=condition)

    def update(self) -> None:
        self._success, self._summary = self.__deviation.update()

    def set_frame(self, msg: DiagnosticArray, module: str) -> None:
        self._frame = self.__deviation.set_frame(msg)
        self.update()

    def set_final_metrics(self) -> None:
        self._frame = {"FinalMetrics": self.__deviation.final_metrics()}
