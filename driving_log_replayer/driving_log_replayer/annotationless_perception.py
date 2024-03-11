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
from typing import ClassVar
from typing import Literal

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from pydantic import BaseModel
from pydantic import field_validator
import simplejson as json

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import number
from driving_log_replayer.scenario import Scenario

OBJECT_CLASSIFICATION_TUPLE = (
    "UNKNOWN",
    "CAR",
    "TRUCK",
    "BUS",
    "TRAILER",
    "MOTORCYCLE",
    "BICYCLE",
    "PEDESTRIAN",
)

OBJECT_CLASSIFICATION = Literal[OBJECT_CLASSIFICATION_TUPLE]


class DiagValue(BaseModel):
    min: float
    max: float
    mean: float


class ClassConditionValue(BaseModel):
    Threshold: dict[str, DiagValue]
    PassRange: tuple[float, float]

    def set_threshold(self, threshold: dict[str, DiagValue]) -> None:
        self.Threshold = threshold

    @field_validator("PassRange", mode="before")
    @classmethod
    def validate_pass_range(cls, v: str) -> tuple[number, number]:
        boundary = 1.0
        s_lower, s_upper = v.split("-")
        lower = float(s_lower)
        upper = float(s_upper)

        if lower > boundary:
            lower_error = f"lower value must be <= {boundary}"
            raise ValueError(lower_error)
        if upper < boundary:
            upper_error = f"upper value must be >= {boundary}"
            raise ValueError(upper_error)
        return (lower, upper)

    def set_pass_range(self, v: str) -> None:
        if v != "":  # skip if launch arg is not set
            self.PassRange = ClassConditionValue.validate_pass_range(v)


class Conditions(BaseModel):
    ClassConditions: dict[OBJECT_CLASSIFICATION, ClassConditionValue]

    def set_threshold_from_file(self, file_path: str) -> None:
        result_file = Path(file_path)
        if file_path != "" and result_file.exists():  # Path("") is current path
            with result_file.open() as f:
                last_line = f.readlines()[-1]
                try:
                    result_json_dict = json.loads(last_line)
                    self.Threshold = result_json_dict["Frame"]["Deviation"]["Metrics"]
                    # ここ考える
                except json.JSONDecodeError:
                    self.Threshold = {}


class Evaluation(BaseModel):
    UseCaseName: Literal["annotationless_perception"]
    UseCaseFormatVersion: Literal["0.2.0"]
    Conditions: Conditions


class AnnotationlessPerceptionScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Deviation(EvaluationItem):
    received_data: dict = field(default_factory=dict)
    # received_data = {lateral_deviation: {min: sum_min, max: sum_max, mean: sum_mean} ... }
    DEFAULT_THRESHOLD: ClassVar[str] = (
        1000.0  # A value that does not overflow when multiplied by a factor of range and is large enough to be a threshold value.
    )
    metrics_dict: dict = field(default_factory=dict)

    @classmethod
    def get_default_condition(cls) -> ClassConditionValue:
        return ClassConditionValue(Threshold={}, PassRange="0.0-1.0")

    def set_frame(self, msg: dict[str, dict]) -> dict:
        self.total += 1
        frame_success = True
        info_dict = {}
        self.metrics_dict = {}
        for status_name, values in msg.items():
            self.add(status_name, values)
            info_dict[status_name] = values
            if self.condition.Threshold.get(status_name) is None:
                self.condition.Threshold[status_name] = DiagValue(
                    min=Deviation.DEFAULT_THRESHOLD,
                    max=Deviation.DEFAULT_THRESHOLD,
                    mean=Deviation.DEFAULT_THRESHOLD,
                )
            self.metrics_dict[status_name], diag_success = self.calc_average_and_success(
                status_name,
                self.condition.Threshold[status_name],
            )
            frame_success = frame_success and diag_success
        self.success = frame_success
        self.summary = f"{self.name} ({self.success_str()})"
        return {
            "Result": {"Total": self.success_str(), "Frame": self.success_str()},
            "Info": info_dict,
            "Metrics": self.metrics_dict,
        }

    def add(self, key: str, values: dict) -> None:
        if self.received_data.get(key) is None:
            self.received_data[key] = {"min": 0.0, "max": 0.0, "mean": 0.0}  # initialize
        self.received_data[key]["min"] += values["min"]
        self.received_data[key]["max"] += values["max"]
        self.received_data[key]["mean"] += values["mean"]

    def calc_average_and_success(self, key: str, threshold: DiagValue) -> tuple[dict, bool]:
        if self.received_data.get(key) is None or self.total == 0:
            return {"min": 0.0, "max": 0.0, "mean": 0.0}
        a_min = self.received_data[key]["min"] / self.total
        a_max = self.received_data[key]["max"] / self.total
        a_mean = self.received_data[key]["mean"] / self.total
        self.condition: Conditions
        lower = self.condition.PassRange[0]
        upper = self.condition.PassRange[1]
        is_success = (
            (threshold.min * lower <= a_min <= threshold.min * upper)
            and (threshold.max * lower <= a_max <= threshold.max * upper)
            and (threshold.mean * lower <= a_mean <= threshold.mean * upper)
        )
        return {"min": a_min, "max": a_max, "mean": a_mean}, is_success


class DeviationClassContainer:
    def __init__(self, condition: Conditions) -> None:
        self.__container: dict[OBJECT_CLASSIFICATION, Deviation] = {}
        for k, v in condition.ClassConditions.items():
            self.__container[k] = Deviation(
                name=k,
                condition=v,
            )

    def set_frame(self, msg: DiagnosticArray) -> dict:
        diag_array_class: dict[OBJECT_CLASSIFICATION, dict[str, dict]] = {}
        frame_result: dict[OBJECT_CLASSIFICATION, dict] = {"Ego": {}}
        # Add diag data separately for each class.
        for diag_status in msg.status:
            diag_status: DiagnosticStatus
            class_name, diag_dict = DeviationClassContainer.get_classname_and_value(diag_status)
            if diag_array_class.get(class_name) is None:
                diag_array_class[class_name] = {}
            diag_array_class[class_name].update(diag_dict)
        # evaluate for each class
        for class_name in diag_array_class:
            if self.__container.get(class_name) is None:
                self.__container[class_name] = Deviation(
                    name=class_name,
                    condition=Deviation.get_default_condition(),
                )
            frame_result[class_name] = self.__container[class_name].set_frame(
                diag_array_class[class_name],
            )
        return frame_result

    @classmethod
    def get_classname_and_value(cls, diag: DiagnosticStatus) -> tuple[str, dict[str, dict]]:
        rtn_class_name = ""
        for class_name in OBJECT_CLASSIFICATION_TUPLE:
            if class_name in diag.name:
                rtn_class_name = class_name
                break
        status_name_removed_class = diag.name.replace(f"_{rtn_class_name}", "")
        values = {value.key: float(value.value) for value in diag.values}  # min, max, mean
        return rtn_class_name, {status_name_removed_class: values}

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = ""
        for evaluation_item in self.__container.values():
            rtn_summary += " " + evaluation_item.summary
            if not evaluation_item.success:
                rtn_success = False
        prefix_str = "Passed:" if rtn_success else "Failed:"
        rtn_summary = prefix_str + rtn_summary
        return (rtn_success, rtn_summary)

    def final_metrics(self) -> dict:
        rtn_dict = {}
        for class_name, evaluation_item in self.__container.items():
            rtn_dict[class_name] = evaluation_item.metrics_dict
        return rtn_dict


class AnnotationlessPerceptionResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__deviation = DeviationClassContainer(condition=condition)

    def update(self) -> None:
        self._success, self._summary = self.__deviation.update()

    def set_frame(self, msg: DiagnosticArray) -> None:
        self._frame = self.__deviation.set_frame(msg)
        self.update()

    def set_final_metrics(self) -> None:
        self._frame = {"FinalMetrics": self.__deviation.final_metrics()}
