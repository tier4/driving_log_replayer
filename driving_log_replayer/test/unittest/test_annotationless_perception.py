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

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from pydantic import ValidationError
import pytest

from driving_log_replayer.annotationless_perception import AnnotationlessPerceptionScenario
from driving_log_replayer.annotationless_perception import ClassConditionValue
from driving_log_replayer.annotationless_perception import Conditions
from driving_log_replayer.annotationless_perception import Deviation
from driving_log_replayer.result import get_sample_result_path
from driving_log_replayer.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    assert scenario.ScenarioName == "sample_annotationless_perception"
    assert scenario.Evaluation.Conditions.ClassConditions["CAR"].Threshold["lateral_deviation"] == {
        "min": 10.0,
        "max": 10.0,
        "mean": 10.0,
    }
    assert scenario.Evaluation.Conditions.ClassConditions["CAR"].PassRange == (0.5, 1.05)


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
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    input_threshold = {
        "min": 0.0007570793650793651,
        "max": 0.01905171904761904,
        "mean": 0.006783434920634924,
    }

    scenario.Evaluation.Conditions.ClassConditions["CAR"].set_threshold(input_threshold)
    threshold = scenario.Evaluation.Conditions.ClassConditions["CAR"].Threshold
    assert threshold["lateral_deviation"] == {
        "min": 0.0007570793650793651,
        "max": 0.01905171904761904,
        "mean": 0.006783434920634924,
    }


def test_set_pass_range() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    scenario.Evaluation.Conditions.set_pass_range("0.4-1.1")
    assert scenario.Evaluation.Conditions.ClassConditions["CAR"].PassRange == (0.4, 1.1)


# @pytest.fixture()
# def create_deviation() -> Deviation:
#     condition = Conditions(
#         Threshold={"lateral_deviation": {"min": 10.0, "max": 10.0, "mean": 10.0}},
#         PassRange="0.5-1.05",
#     )
#     return Deviation(
#         condition=condition,
#         total=9,
#         received_data={"lateral_deviation": {"min": 49.0, "max": 100.0, "mean": 70.0}},
#     )


# def test_deviation_success(create_deviation: Callable) -> None:
#     evaluation_item: Deviation = create_deviation
#     status = DiagnosticStatus(
#         name="lateral_deviation",
#         values=[
#             KeyValue(key="min", value="1.0"),
#             KeyValue(key="max", value="1.0"),
#             KeyValue(key="mean", value="1.0"),
#         ],
#     )
#     evaluation_item.set_frame(DiagnosticArray(status=[status]))
#     assert evaluation_item.success is True


# def test_deviation_fail_upper_limit(create_deviation: Callable) -> None:
#     evaluation_item: Deviation = create_deviation
#     status = DiagnosticStatus(
#         name="lateral_deviation",
#         values=[
#             KeyValue(key="min", value="10.0"),
#             KeyValue(key="max", value="10.0"),  # 110 / 10 = 11 > 10.5
#             KeyValue(key="mean", value="10.0"),
#         ],
#     )
#     evaluation_item.set_frame(DiagnosticArray(status=[status]))
#     assert evaluation_item.success is False


# def test_deviation_fail_lower_limit(create_deviation: Callable) -> None:
#     evaluation_item: Deviation = create_deviation
#     status = DiagnosticStatus(
#         name="lateral_deviation",
#         values=[
#             KeyValue(key="min", value="0.0"),  # 49.0 / 10 = 4.9 < 5.0
#             KeyValue(key="max", value="1.0"),
#             KeyValue(key="mean", value="1.0"),
#         ],
#     )
#     evaluation_item.set_frame(DiagnosticArray(status=[status]))
#     assert evaluation_item.success is False
