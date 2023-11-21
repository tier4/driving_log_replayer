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
from typing import ClassVar

from diagnostic_msgs.msg import DiagnosticArray
from pydantic import BaseModel
from typing_extensions import Literal

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import InitialPose
from driving_log_replayer.scenario import Scenario


class Evaluation(BaseModel):
    UseCaseName: Literal["yabloc"]
    UseCaseFormatVersion: Literal["0.1.0"]
    InitialPose: InitialPose | None


class YablocScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Availability(EvaluationItem):
    name: str = "Yabloc Availability"
    TARGET_DIAG_NAME: ClassVar[str] = "yabloc_monitor: yabloc_status"

    def set_frame(self, msg: DiagnosticArray) -> dict:
        include_target_status = False
        for diag_status in msg.status:
            if diag_status.name != Availability.TARGET_DIAG_NAME:
                continue
            include_target_status = True
            values = {value.key: value.value for value in diag_status.values}
            status_str = values.get("Availability", "Availability Key Not Found")  # avoid KeyError
            self.success = status_str == "OK"
            self.summary = f"{self.name} ({self.success_str()}): {status_str}"
            break
        if include_target_status:
            return {
                "Ego": {},
                "Availability": {
                    "Result": {"Total": self.success_str(), "Frame": self.success_str()},
                    "Info": {},
                },
            }
        return {
            "Ego": {},
            "Availability": {
                "Result": {"Total": self.success_str(), "Frame": "Warn"},
                "Info": {"Reason": "diagnostics does not contain yabloc_status"},
            },
        }


class YabLocResult(ResultBase):
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

    def set_frame(self, msg: DiagnosticArray) -> None:
        self._frame = self.__availability.set_frame(msg)
        self.update()
