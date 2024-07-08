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

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario


class Evaluation(BaseModel):
    UseCaseName: Literal["ar_tag_based_localizer"]
    UseCaseFormatVersion: Literal["1.0.0"]
    Datasets: list[dict]


class ArtagBasedLocalizerScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Availability(EvaluationItem):
    name: str = "ArTagBasedLocalizer Availability"

    def set_frame(self, diag_status: DiagnosticStatus) -> dict:
        values = {value.key: value.value for value in diag_status.values}
        detection_count = int(values.get("Number of Detected AR Tags", "-1"))

        # If the number detected is greater than or equal to 0, it means that the system has worked properly.
        if detection_count >= 0:
            self.success = True
            self.summary = f"{self.name} ({self.success_str()}): Detected {detection_count} AR tags"
        else:
            self.success = False
            self.summary = f"{self.name} ({self.success_str()}): Illegal value"
        return {
            "Ego": {},
            "Availability": {
                "Result": {"Total": self.success_str(), "Frame": self.success_str()},
                "Info": {},
            },
        }


class ArTagBasedLocalizerResult(ResultBase):
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
