# Copyright (c) 2023 TIER IV.inc
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
from typing import Literal

from diagnostic_msgs.msg import DiagnosticStatus
from pydantic import BaseModel

from log_evaluator.result import EvaluationItem
from log_evaluator.result import ResultBase
from log_evaluator.scenario import Scenario


class Evaluation(BaseModel):
    UseCaseName: Literal["eagleye"]
    UseCaseFormatVersion: Literal["1.0.0"]
    Datasets: list[dict]


class EagleyeScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Availability(EvaluationItem):
    name: str = "Eagleye Availability"

    def set_frame(self, diag_status: DiagnosticStatus) -> dict:
        self.success = diag_status.level == DiagnosticStatus.OK
        self.summary = f"{self.name} ({self.success_str()}): {diag_status.message}"
        return {
            "Ego": {},
            "Availability": {
                "Result": {"Total": self.success_str(), "Frame": self.success_str()},
                "Info": {},
            },
        }


class EagleyeResult(ResultBase):
    def __init__(self, condition) -> None:  # noqa
        # condition is for future extensibility and code commonality
        super().__init__()
        self.__availability = Availability()

    def update(self) -> None:
        summary_str = f"{self.__availability.summary}"
        if self.__availability.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(self, diag_status: DiagnosticStatus) -> None:
        self._frame = self.__availability.set_frame(diag_status)
        self.update()
