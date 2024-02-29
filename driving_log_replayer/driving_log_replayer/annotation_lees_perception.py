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
from typing import Literal

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from pydantic import BaseModel
from pydantic import field_validator
import simplejson as json

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario


class Criteria(BaseModel):
    Threshold: dict

    @field_validator("Threshold")
    def create_default_threshold(cls) -> dict:
        return {}

    def set_threshold_from_arg(self, arg: str) -> None:
        self.Threshold = json.loads(arg)


class Conditions(BaseModel):
    LaunchSensing: bool
    Criteria: Criteria


class Evaluation(BaseModel):
    UseCaseName: Literal["annotation_less_perception"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Conditions: Conditions


class AnnotationLessPerceptionScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Deviation(EvaluationItem):
    name: str = "AnnotationLessPerception Deviation"
    success: bool = True

    def set_frame(self, msg: DiagnosticArray) -> dict:
        self.summary = "Passed"
        rtn_dict = {"Ego": {}, "Result": {"Total": self.success_str(), "Frame": "Success"}}
        for diag_status in msg.status:
            diag_status: DiagnosticStatus
            values = {
                value.key: float(value.value) for value in diag_status.values
            }  # min, max, mean
            rtn_dict[diag_status.name] = values
        return rtn_dict


class AnnotationLessPerceptionResult(ResultBase):
    def __init__(self, condition) -> None:  # noqa
        # condition is for future extensibility and code commonality
        super().__init__()
        self.__deviation = Deviation()

    def update(self) -> None:
        summary_str = f"{self.__deviation.summary}"
        if self.__deviation.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(self, msg: DiagnosticArray) -> None:
        self._frame = self.__deviation.set_frame(msg)
        self.update()
