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
from pydantic import field_validator
import simplejson as json

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
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

METRICS_KEY_TUPLE = ("min", "max", "mean", "metric_value")
METRICS_KEY = Literal[METRICS_KEY_TUPLE]


class DiagValue(BaseModel):
    min: float | None = None
    max: float | None = None
    mean: float | None = None
    metric_value: float | None = None


class ClassConditionValue(BaseModel):
    Threshold: dict[str, DiagValue]
    PassRange: dict[METRICS_KEY, tuple[float, float]]

    @field_validator("PassRange", mode="before")
    @classmethod
    def validate_pass_range(cls, range_dict: dict) -> dict[METRICS_KEY, tuple[float, float]]:
        rtn_dict = {}
        for k, v in range_dict.items():
            if k not in METRICS_KEY_TUPLE:
                key_error = "pass_range key must be 'min', 'max', 'mean', and 'metric_value'"
                raise ValueError(key_error)
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
            rtn_dict[k] = (lower, upper)
        return rtn_dict

    @classmethod
    def get_default_condition(cls) -> "ClassConditionValue":
        return ClassConditionValue(
            Threshold={},
            PassRange={
                "min": "0.0-1.0",
                "max": "0.0-1.0",
                "mean": "0.5-1.0",
                "metric_value": "0.9-1.1",
            },
        )

    def set_threshold(self, threshold_dict: dict[str, dict]) -> None:
        threshold_diag: dict[str, DiagValue] = {}
        for k, v in threshold_dict.items():
            threshold_diag[k] = DiagValue(**v)
        self.Threshold = threshold_diag

    def update_threshold(self, threshold_dict: dict[str, dict]) -> None:
        for k, v in self.Threshold.items():
            if threshold_dict.get(k) is None:
                continue
            diag_value = DiagValue(**threshold_dict[k])
            if v.min is None:
                diag_value.min = None
            if v.max is None:
                diag_value.max = None
            if v.mean is None:
                diag_value.mean = None
            if v.metric_value is None:
                diag_value.metric_value = None
            self.Threshold[k] = diag_value

    def set_pass_range(self, v: dict) -> None:
        if v != "":  # skip if launch arg is not set
            self.PassRange = ClassConditionValue.validate_pass_range(v)


class Conditions(BaseModel):
    ClassConditions: dict[OBJECT_CLASSIFICATION, ClassConditionValue]

    def update_threshold_from_file(self, file_path: str) -> None:
        # this method update condition only in scenario
        final_metrics = Conditions.load_final_metrics(file_path)
        if final_metrics is None:
            return
        for class_name in self.ClassConditions:
            if final_metrics.get(class_name) is not None:
                self.ClassConditions[class_name].update_threshold(final_metrics[class_name])

    @classmethod
    def load_final_metrics(cls, file_path: str) -> dict | None:
        result_file = Path(file_path)
        if file_path == "" or not result_file.exists():  # Path("") is current path
            return None
        with result_file.open() as f:
            last_line = f.readlines()[-1]
        try:
            result_json_dict = json.loads(last_line)
            return result_json_dict["Frame"]["FinalMetrics"]
        except json.JSONDecodeError:
            return None

    def set_pass_range(self, v: str) -> None:
        if v == "":  # skip if launch arg is not set
            return
        try:
            range_dict = json.loads(v)
            for class_name, range_str in range_dict.items():
                if self.ClassConditions.get(class_name) is None:
                    self.ClassConditions[class_name] = ClassConditionValue.get_default_condition()
                self.ClassConditions[class_name].set_pass_range(range_str)
        except json.JSONDecodeError:
            pass


class Evaluation(BaseModel):
    UseCaseName: Literal["annotationless_perception"]
    UseCaseFormatVersion: Literal["0.3.0"]
    Conditions: Conditions


class AnnotationlessPerceptionScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class ObjectMetrics(EvaluationItem):
    received_data: dict = field(default_factory=dict)
    # received_data = {lateral_deviation: {min: minimum_min, max: maximum_max, mean: sum_mean, total_sum: count_sum} ... }
    metrics_dict: dict = field(default_factory=dict)
    fail_details: dict = field(default_factory=dict)

    def set_frame(self, msg: dict[str, dict]) -> dict:
        self.condition: ClassConditionValue
        frame_success = True
        info_dict = {}
        frame_fail_items = ""
        for status_name, values in msg.items():
            info_dict[status_name] = values
            metrics_dict, diag_success = self.calc_metrics_and_success(
                status_name,
                values,
            )
            if metrics_dict != {}:
                self.metrics_dict[status_name] = metrics_dict
            if diag_success is False:
                frame_fail_items += f", {status_name}"
            frame_success = frame_success and diag_success
        self.success = frame_success
        if self.condition.Threshold == {}:  # not evaluation target
            self.summary = f"{self.name} (NotTested)"
        else:
            self.summary = f"{self.name} ({self.success_str()}{frame_fail_items})"
        return {
            "Result": {"Total": self.success_str(), "Frame": self.success_str()},
            "Info": info_dict,
            "Metrics": self.metrics_dict,
        }

    def calc_metrics_and_success(  # noqa
        self,
        key: str,
        values: dict,
    ) -> tuple[dict, bool]:
        threshold_key: DiagValue | None = self.condition.Threshold.get(key)
        pass_range = self.condition.PassRange
        # metric_value type
        if "metric_value" in values:
            if threshold_key is None or threshold_key.metric_value is None:
                return {}, True
            metric_value_success = (
                threshold_key.metric_value * pass_range["metric_value"][0]
                <= values["metric_value"]
                <= threshold_key.metric_value * pass_range["metric_value"][1]
            )
            return {}, metric_value_success

        # min, max, mean type
        # initialize
        if self.received_data.get(key) is None:
            self.received_data[key] = {
                "min": sys.float_info.max,
                "max": 0.0,
                "mean": 0.0,
                "total_sum": 0,
            }
        rdk = self.received_data[key]
        rdk["total_sum"] += 1
        rdk["min"] = min(rdk["min"], values["min"])
        rdk["max"] = max(rdk["max"], values["max"])
        rdk["mean"] += values["mean"]
        # calc metrics
        a_mean = rdk["mean"] / rdk["total_sum"]

        # evaluate
        if threshold_key is None:
            is_success = True  # Calculate metrics only, return always True
        else:
            is_success_min = True
            is_success_max = True
            is_success_mean = True  # if threshold mean is not set
            if threshold_key.min is not None:
                # Once min_success is false, it is not calculated thereafter.
                is_success_min = (
                    threshold_key.min * pass_range["min"][0]
                    <= rdk["min"]
                    <= threshold_key.min * pass_range["min"][1]
                )
            if threshold_key.max is not None:
                # Once max_success is false, it is not calculated thereafter.
                is_success_max = (
                    threshold_key.max * pass_range["max"][0]
                    <= rdk["max"]
                    <= threshold_key.max * pass_range["max"][1]
                )
            if threshold_key.mean is not None:
                is_success_mean = (
                    threshold_key.mean * pass_range["mean"][0]
                    <= a_mean
                    <= threshold_key.mean * pass_range["mean"][1]
                )
            is_success = is_success_min and is_success_max and is_success_mean

            # Store the measured value and threshold value for
            if not is_success_min:
                self.fail_details[key + "_min"] = (
                    rdk["min"],
                    threshold_key.min * pass_range["min"][0],
                    threshold_key.min * pass_range["min"][1],
                )
            if not is_success_max:
                self.fail_details[key + "_max"] = (
                    rdk["max"],
                    threshold_key.max * pass_range["max"][0],
                    threshold_key.max * pass_range["max"][1],
                )
            if is_success_mean:
                # If mean is successful, remove it from fail_details if it exists
                self.fail_details.pop(key + "_mean", None)
            else:
                self.fail_details[key + "_mean"] = (
                    a_mean,
                    threshold_key.mean * pass_range["mean"][0],
                    threshold_key.mean * pass_range["mean"][1],
                )
        return {"min": rdk["min"], "max": rdk["max"], "mean": a_mean}, is_success


class ObjectMetricsClassContainer:
    def __init__(self, condition: Conditions) -> None:
        self.__container: dict[OBJECT_CLASSIFICATION, ObjectMetrics] = {}
        for k, v in condition.ClassConditions.items():
            self.__container[k] = ObjectMetrics(
                name=k,
                condition=v,
            )

    def set_frame(self, msg: DiagnosticArray) -> dict:
        diag_array_class: dict[OBJECT_CLASSIFICATION, dict[str, dict]] = {}
        frame_result: dict[OBJECT_CLASSIFICATION, dict] = {}
        # Add diag data separately for each class.
        for diag_status in msg.status:
            diag_status: DiagnosticStatus
            class_name, diag_dict = ObjectMetricsClassContainer.get_classname_and_value(diag_status)
            if diag_array_class.get(class_name) is None:
                diag_array_class[class_name] = {}
            diag_array_class[class_name].update(diag_dict)
        # evaluate for each class
        for class_name in diag_array_class:
            if self.__container.get(class_name) is None:
                # Add default ObjectMetricsClass for metrics aggregation
                self.__container[class_name] = ObjectMetrics(
                    name=class_name,
                    condition=ClassConditionValue.get_default_condition(),
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
        values = {
            value.key: float(value.value) for value in diag.values
        }  # min, max, mean, metric_value
        return rtn_class_name, {status_name_removed_class: values}

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = []
        for class_name, evaluation_item in self.__container.items():
            if not evaluation_item.success:
                rtn_success = False
                fail_info = "\n\t".join(
                    [
                        f"{key}: actual_value={val[0]}, expected_value=({val[1]}-{val[2]})"
                        for key, val in evaluation_item.fail_details.items()
                    ],
                )
                rtn_summary.append(f"{class_name} (Fail):\n\t{fail_info}")
            else:
                rtn_summary.append(f"{class_name} (Success)")

        prefix_str = "Passed" if rtn_success else "Failed"
        rtn_summary_str = prefix_str + ":\n" + "\n".join(rtn_summary)
        return (rtn_success, rtn_summary_str)

    def final_metrics(self) -> dict:
        rtn_dict = {}
        for class_name, evaluation_item in self.__container.items():
            rtn_dict[class_name] = evaluation_item.metrics_dict
        return rtn_dict


class AnnotationlessPerceptionResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__obj_container = ObjectMetricsClassContainer(condition=condition)

    def update(self) -> None:
        self._success, self._summary = self.__obj_container.update()

    def set_frame(self, msg: DiagnosticArray) -> None:
        self._frame = self.__obj_container.set_frame(msg)
        self.update()

    def set_final_metrics(self) -> None:
        self._frame = {"FinalMetrics": self.__obj_container.final_metrics()}
