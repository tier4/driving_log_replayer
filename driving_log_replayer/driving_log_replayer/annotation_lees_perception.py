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
import sys
from typing import Literal

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from pydantic import BaseModel
import simplejson as json

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario


class Conditions(BaseModel):
    Threshold: dict

    def set_threshold_from_file(self, file_path: str) -> None:
        result_file = Path(file_path)
        if file_path != "" and result_file.exists():  # Path("") is current path
            with result_file.open() as f:
                last_line = f.readlines()[-1]
                try:
                    result_json_dict = json.loads(last_line)
                    self.Threshold = result_json_dict["Frame"]["Deviation"]["Metrics"]
                except json.JSONDecodeError:
                    self.Threshold = {}


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
    received_data: dict = field(default_factory=dict)
    # received_data = {lateral_deviation: {min: sum_min, max: sum_max, mean: sum_mean} ... }

    def set_frame(self, msg: DiagnosticArray) -> dict:
        self.total += 1
        frame_success = True
        info_dict = {}
        metrics_dict = {}
        for diag_status in msg.status:
            diag_status: DiagnosticStatus
            values = {
                value.key: float(value.value) for value in diag_status.values
            }  # min, max, mean
            self.add(diag_status.name, values)
            info_dict[diag_status.name] = values
            threshold = self.condition.Threshold.get(
                diag_status.name,
                {
                    "min": sys.float_info.max,
                    "max": sys.float_info.max,
                    "mean": sys.float_info.max,
                },
            )
            metrics_dict[diag_status.name], diag_success = self.calc_average_and_success(
                diag_status.name,
                threshold,
            )
            frame_success = frame_success and diag_success
        self.success = frame_success
        self.summary = f"{self.name} ({self.success_str()})"
        return {
            "Ego": {},
            "Deviation": {
                "Result": {"Total": self.success_str(), "Frame": self.success_str()},
                "Info": info_dict,
                "Metrics": metrics_dict,
            },
        }

    def add(self, key: str, values: dict) -> None:
        if self.received_data.get(key) is None:
            self.received_data[key] = {"min": 0.0, "max": 0.0, "mean": 0.0}  # initialize
        self.received_data[key]["min"] += values["min"]
        self.received_data[key]["max"] += values["max"]
        self.received_data[key]["mean"] += values["mean"]

    def calc_average_and_success(self, key: str, threshold: dict) -> tuple[dict, bool]:
        if self.received_data.get(key) is None or self.total == 0:
            return {"min": 0.0, "max": 0.0, "mean": 0.0}
        a_min = self.received_data[key]["min"] / self.total
        a_max = self.received_data[key]["max"] / self.total
        a_mean = self.received_data[key]["mean"] / self.total
        is_success = (
            (a_min <= threshold["min"])
            and (a_max <= threshold["max"])
            and (a_mean <= threshold["mean"])
        )
        return {"min": a_min, "max": a_max, "mean": a_mean}, is_success


class AnnotationLessPerceptionResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__deviation = Deviation(condition=condition)

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
