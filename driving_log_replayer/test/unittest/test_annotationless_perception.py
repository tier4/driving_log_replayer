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

from driving_log_replayer.annotationless_perception import AnnotationlessPerceptionScenario
from driving_log_replayer.annotationless_perception import ClassConditionValue
from driving_log_replayer.annotationless_perception import Deviation
from driving_log_replayer.annotationless_perception import DiagValue
from driving_log_replayer.result import get_sample_result_path
from driving_log_replayer.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    assert scenario.ScenarioName == "sample_annotationless_perception"
    assert scenario.Evaluation.Conditions.ClassConditions["BUS"].Threshold == {
        "lateral_deviation": DiagValue(max=0.050),
    }
    assert scenario.Evaluation.Conditions.ClassConditions["BUS"].PassRange == {
        "min": (0.0, 2.0),
        "max": (0.0, 2.0),
        "mean": (0.5, 2.0),
    }


def test_range_validation_upper_limit() -> None:
    with pytest.raises(ValidationError):
        ClassConditionValue(
            Threshold={},
            PassRange={"min": "0.0-0.95", "max": "0.0-2.0", "mean": "0.5-2.0"},
        )


def test_range_validation_lower_limit() -> None:
    with pytest.raises(ValidationError):
        ClassConditionValue(
            Threshold={},
            PassRange={"min": "1.1-2.0", "max": "0.0-2.0", "mean": "0.5-2.0"},
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
        "total_objects_count_r50.00_h10.00": DiagValue(max=1.0),
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
def create_deviation() -> Deviation:
    condition = ClassConditionValue(
        Threshold={"lateral_deviation": DiagValue(min=1.0, max=10.0, mean=5.0)},
        PassRange={"min": "0.2-1.05", "max": "0.0-1.05", "mean": "0.95-1.05"},
    )
    return Deviation(
        name="CAR",
        condition=condition,
        total=9,
        received_data={"lateral_deviation": {"min": 1.0, "max": 0.0, "mean": 45.0}},
        min_success={"lateral_deviation": True},
        max_success={"lateral_deviation": True},
    )


def test_deviation_success(create_deviation: Callable) -> None:
    evaluation_item: Deviation = create_deviation
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 10.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is True


def test_deviation_fail_lower_limit_min(create_deviation: Callable) -> None:
    evaluation_item: Deviation = create_deviation
    status_dict = {"lateral_deviation": {"min": 0.1, "max": 10.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_upper_limit_max(create_deviation: Callable) -> None:
    evaluation_item: Deviation = create_deviation
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 12.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_upper_limit_mean(create_deviation: Callable) -> None:
    evaluation_item: Deviation = create_deviation
    # sum_mean = 45.0 + 8.0 = 53.0 average = 53.0 / 10 = 5.3 > 5.0*1.05 = 5.25
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 10.0, "mean": 8.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_lower_limit(create_deviation: Callable) -> None:
    evaluation_item: Deviation = create_deviation
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
