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

from collections.abc import Callable
from pathlib import Path
from typing import Literal

from pydantic import ValidationError
import pytest

from log_evaluator.annotationless_perception import AnnotationlessPerceptionScenario
from log_evaluator.annotationless_perception import ClassConditionValue
from log_evaluator.annotationless_perception import DiagValue
from log_evaluator.annotationless_perception import ObjectMetrics
from log_evaluator.result import get_sample_result_path
from log_evaluator.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    assert scenario.ScenarioName == "sample_annotationless_perception"
    assert scenario.Evaluation.Conditions.ClassConditions["BUS"].Threshold == {
        "yaw_rate": DiagValue(max=0.05),
    }
    assert scenario.Evaluation.Conditions.ClassConditions["BUS"].PassRange == {
        "min": (0.0, 2.0),
        "max": (0.0, 2.0),
        "mean": (0.5, 2.0),
        "metric_value": (0.9, 1.1),
    }


def test_range_validation_upper_limit() -> None:
    with pytest.raises(ValidationError):
        ClassConditionValue(
            Threshold={},
            PassRange={
                "min": "0.0-0.95",
                "max": "0.0-2.0",
                "mean": "0.5-2.0",
                "metric_value": "0.9-1.1",
            },
        )


def test_range_validation_lower_limit() -> None:
    with pytest.raises(ValidationError):
        ClassConditionValue(
            Threshold={},
            PassRange={
                "min": "1.1-2.0",
                "max": "0.0-2.0",
                "mean": "0.5-2.0",
                "metric_value": "0.9-1.1",
            },
        )


def test_path_with_empty_str() -> None:
    f = Path("")  # noqa
    assert f.exists()


def test_set_threshold() -> None:
    class_cond = ClassConditionValue.get_default_condition()
    input_threshold = {
        "lateral_deviation": {
            "min": 0.0007570793650793651,
            "max": 0.01905171904761904,
            "mean": 0.006783434920634924,
        },
    }

    class_cond.set_threshold(input_threshold)
    assert class_cond.Threshold["lateral_deviation"] == DiagValue(
        min=0.0007570793650793651,
        max=0.01905171904761904,
        mean=0.006783434920634924,
    )


def test_update_threshold_from_file() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    scenario.Evaluation.Conditions.update_threshold_from_file(
        get_sample_result_path("annotationless_perception", "result.jsonl").as_posix(),
    )
    # update_threshold_from_file update only keys written in scenario.yaml
    bus_threshold = scenario.Evaluation.Conditions.ClassConditions["BUS"].Threshold
    assert bus_threshold == {
        "yaw_rate": DiagValue(max=0.077258),
    }


def test_set_pass_range() -> None:
    class_cond = ClassConditionValue.get_default_condition()
    class_cond.set_pass_range({"min": "0.0-1.1", "max": "0.0-1.2", "mean": "0.4-1.3"})
    assert class_cond.PassRange == {"min": (0.0, 1.1), "max": (0.0, 1.2), "mean": (0.4, 1.3)}


def test_set_pass_range_from_launch_arg() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    scenario.Evaluation.Conditions.set_pass_range(
        '{"CAR":{"min":"0.3-1.2","max":"0.3-1.2","mean":"0.3-1.2"},"BUS":{"max":"0.2-1.3"}}',
    )
    cond = scenario.Evaluation.Conditions
    assert cond.ClassConditions["CAR"].PassRange == {
        "min": (0.3, 1.2),
        "max": (0.3, 1.2),
        "mean": (0.3, 1.2),
    }
    assert cond.ClassConditions["BUS"].PassRange == {"max": (0.2, 1.3)}


@pytest.fixture()
def create_obj_metrics() -> ObjectMetrics:
    condition = ClassConditionValue(
        Threshold={"lateral_deviation": DiagValue(min=1.0, max=10.0, mean=5.0)},
        PassRange={"min": "0.2-1.05", "max": "0.0-1.05", "mean": "0.95-1.05"},
    )
    return ObjectMetrics(
        name="CAR",
        condition=condition,
        received_data={"lateral_deviation": {"min": 1.0, "max": 0.0, "mean": 45.0, "total_sum": 9}},
    )


def test_deviation_success(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 10.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is True


def test_deviation_fail_lower_limit_min(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    status_dict = {"lateral_deviation": {"min": 0.1, "max": 10.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_upper_limit_max(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 12.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_upper_limit_mean(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    # sum_mean = 45.0 + 8.0 = 53.0 average = 53.0 / 10 = 5.3 > 5.0*1.05 = 5.25
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 10.0, "mean": 8.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_lower_limit(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    # sum_mean = 45.0 + 2.0 = 47.0 average = 47.0 / 10 = 4.7 < 5.0*0.95 = 4.75
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 10.0, "mean": 2.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_create_literal_from_tuple() -> None:
    class_tuple = (
        "UNKNOWN",
        "CAR",
        "TRUCK",
        "BUS",
        "TRAILER",
        "MOTORCYCLE",
        "BICYCLE",
        "PEDESTRIAN",
    )
    class_literal = Literal[
        "UNKNOWN",
        "CAR",
        "TRUCK",
        "BUS",
        "TRAILER",
        "MOTORCYCLE",
        "BICYCLE",
        "PEDESTRIAN",
    ]

    literal_using_tuple = Literal[class_tuple]
    assert literal_using_tuple == class_literal
