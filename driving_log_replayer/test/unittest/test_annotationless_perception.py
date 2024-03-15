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
    assert scenario.Evaluation.Conditions.ClassConditions["BUS"].Threshold[
        "lateral_deviation"
    ] == DiagValue(max=10.0)
    assert scenario.Evaluation.Conditions.ClassConditions["BUS"].PassRange == (0.5, 1.05)


def test_range_validation_upper_limit() -> None:
    with pytest.raises(ValidationError):
        ClassConditionValue(Threshold={}, PassRange=("0.5-0.95"))


def test_range_validation_lower_limit() -> None:
    with pytest.raises(ValidationError):
        ClassConditionValue(Threshold={}, PassRange=("1.05-1.5"))


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


def test_set_threshold_from_file() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    scenario.Evaluation.Conditions.set_threshold_from_file(
        get_sample_result_path("annotationless_perception", "result.jsonl").as_posix(),
    )
    # set_threshold_from_file completely override condition from result.jsonl
    # condition on TRUCK is not specified in sample scenario.yaml
    truck_threshold = scenario.Evaluation.Conditions.ClassConditions["TRUCK"].Threshold
    assert truck_threshold["lateral_deviation"] == DiagValue(
        min=0.013153343750000001,
        max=0.013153343750000001,
        mean=0.013153343750000001,
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
        "lateral_deviation": DiagValue(max=0.002148912891986063),
    }


def test_set_pass_range() -> None:
    class_cond = ClassConditionValue.get_default_condition()
    class_cond.set_pass_range("0.4-1.1")
    assert class_cond.PassRange == (0.4, 1.1)


def test_set_pass_range_from_launch_arg() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    scenario.Evaluation.Conditions.set_pass_range('{"CAR":"0.3-1.2","BUS":"0.2-1.3"}')
    cond = scenario.Evaluation.Conditions
    assert cond.ClassConditions["CAR"].PassRange == (0.3, 1.2)
    assert cond.ClassConditions["BUS"].PassRange == (0.2, 1.3)


@pytest.fixture()
def create_deviation() -> Deviation:
    condition = ClassConditionValue(
        Threshold={"lateral_deviation": DiagValue(min=10.0, max=10.0)},
        PassRange="0.5-1.05",
    )
    return Deviation(
        name="CAR",
        condition=condition,
        total=9,
        received_data={"lateral_deviation": {"min": 49.0, "max": 100.0, "mean": 100.0}},
    )


def test_deviation_success(create_deviation: Callable) -> None:
    evaluation_item: Deviation = create_deviation
    # mean 110 / 10 = 11. but Mean is not subject to evaluation
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 1.0, "mean": 10.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is True


def test_deviation_fail_upper_limit(create_deviation: Callable) -> None:
    evaluation_item: Deviation = create_deviation
    # max 110 / 10 = 11 > 10.5
    status_dict = {"lateral_deviation": {"min": 10.0, "max": 10.0, "mean": 10.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_lower_limit(create_deviation: Callable) -> None:
    evaluation_item: Deviation = create_deviation
    # min 49.0 / 10 = 4.9 < 5.0
    status_dict = {"lateral_deviation": {"min": 0.0, "max": 1.0, "mean": 1.0}}
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
