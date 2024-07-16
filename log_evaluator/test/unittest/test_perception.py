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
from collections.abc import Callable
import sys

from perception_eval.common import DynamicObject
from perception_eval.common.dataset import FrameGroundTruth
from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.common.label import AutowareLabel
from perception_eval.common.label import Label
from perception_eval.common.schema import FrameID
from perception_eval.common.shape import Shape
from perception_eval.common.shape import ShapeType
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation import DynamicObjectWithPerceptionResult
from perception_eval.evaluation import PerceptionFrameResult
from perception_eval.evaluation.metrics import MetricsScoreConfig
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from pyquaternion import Quaternion
import pytest

from log_evaluator.perception import Criteria
from log_evaluator.perception import Filter
from log_evaluator.perception import Perception
from log_evaluator.perception import PerceptionScenario
from log_evaluator.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    assert scenario.Evaluation.Conditions.Criterion[0].CriteriaMethod == "num_gt_tp"
    assert scenario.Evaluation.Conditions.Criterion[1].CriteriaLevel == "easy"


def test_scenario_criteria_custom_level() -> None:
    scenario: PerceptionScenario = load_sample_scenario(
        "perception",
        PerceptionScenario,
        "scenario.criteria.custom.yaml",
    )
    assert scenario.Evaluation.Conditions.Criterion[0].CriteriaMethod == [
        "metrics_score",
        "metrics_score_maph",
    ]
    assert scenario.Evaluation.Conditions.Criterion[0].CriteriaLevel == [10.0, 10.0]
    assert scenario.Evaluation.Conditions.Criterion[0].Filter.Distance is None


def test_filter_distance_omit_upper_limit() -> None:
    filter_condition = Filter(Distance="1.0-")
    assert filter_condition.Distance[0] == 1.0  # noqa
    assert filter_condition.Distance[1] == sys.float_info.max


def test_filter_distance_is_not_number() -> None:
    with pytest.raises(ValueError):  # noqa
        Filter(Distance="a-b")


def test_filter_distance_element_is_not_two() -> None:
    with pytest.raises(ValueError):  # noqa
        Filter(Distance="1.0-2.0-3.0")


def test_filter_distance_min_max_reversed() -> None:
    with pytest.raises(ValueError):  # noqa
        Filter(Distance="2.0-1.0")


@pytest.fixture()
def create_frame_result() -> PerceptionFrameResult:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    evaluation_config_dict = scenario.Evaluation.PerceptionEvaluationConfig[
        "evaluation_config_dict"
    ]
    evaluation_config_dict["label_prefix"] = "autoware"
    m_params: dict = {
        "target_labels": evaluation_config_dict["target_labels"],
        "center_distance_thresholds": evaluation_config_dict.get("center_distance_thresholds"),
        "plane_distance_thresholds": evaluation_config_dict.get("plane_distance_thresholds"),
        "iou_2d_thresholds": evaluation_config_dict.get("iou_2d_thresholds"),
        "iou_3d_thresholds": evaluation_config_dict.get("iou_3d_thresholds"),
    }
    evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
        dataset_paths=["/tmp/dlr"],  # noqa
        frame_id="base_link",
        result_root_directory="/tmp/dlr/result/{TIME}",  # noqa
        evaluation_config_dict=evaluation_config_dict,
        load_raw_data=False,
    )

    return PerceptionFrameResult(
        object_results=[],
        frame_ground_truth=FrameGroundTruth(123, "12", []),
        metrics_config=MetricsScoreConfig(
            EvaluationTask.DETECTION,
            **m_params,
        ),
        critical_object_filter_config=CriticalObjectFilterConfig(
            evaluation_config,
            evaluation_config_dict["target_labels"],
            max_x_position_list=[30.0, 30.0, 30.0, 30.0],
            max_y_position_list=[30.0, 30.0, 30.0, 30.0],
        ),
        frame_pass_fail_config=PerceptionPassFailConfig(
            evaluation_config,
            evaluation_config_dict["target_labels"],
        ),
        unix_time=123,
        target_labels=[AutowareLabel.CAR],
    )


@pytest.fixture()
def create_tp_normal() -> Perception:
    return Perception(
        name="criteria0",
        condition=Criteria(
            PassRate=95.0,
            CriteriaMethod="num_tp",
            CriteriaLevel="normal",
            Filter=Filter(Distance=None),
        ),
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_tp_hard() -> Perception:
    return Perception(
        name="criteria0",
        condition=Criteria(
            PassRate=95.0,
            CriteriaMethod="num_tp",
            CriteriaLevel="hard",
            Filter=Filter(Distance=None),
        ),
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_dynamic_object() -> DynamicObjectWithPerceptionResult:
    dynamic_obj = DynamicObject(
        123,
        FrameID.BASE_LINK,
        (1.0, 2.0, 3.0),
        Quaternion(),
        Shape(ShapeType.BOUNDING_BOX, (1.0, 1.0, 1.0)),
        (1.0, 2.0, 3.0),
        0.5,
        Label(AutowareLabel.CAR, "car"),
    )
    return DynamicObjectWithPerceptionResult(dynamic_obj, None, True)  # noqa


def test_perception_fail_has_no_object(
    create_tp_normal: Callable,
    create_frame_result: Callable,
) -> None:
    evaluation_item: Perception = create_tp_normal
    result: PerceptionFrameResult = create_frame_result
    # add no tp_object_results, fp_object_results
    frame_dict = evaluation_item.set_frame(result)
    # check total is not changed (skip count)
    assert evaluation_item.total == 99  # noqa
    assert evaluation_item.success is True  # default is True
    assert frame_dict == {"NoGTNoObj": 1}


def test_perception_success_tp_normal(
    create_tp_normal: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
) -> None:
    evaluation_item: Perception = create_tp_normal
    result: PerceptionFrameResult = create_frame_result
    tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    result.pass_fail_result.tp_object_results = tp_objects_results
    result.pass_fail_result.fp_object_results = fp_objects_results
    # score 50.0 >= NORMAL(50.0)
    frame_dict = evaluation_item.set_frame(result)
    assert evaluation_item.success is True
    assert evaluation_item.summary == "criteria0 (Success): 95 / 100 -> 95.00%"
    assert frame_dict["PassFail"] == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "TP": "5 [car, car, car, car, car]",
            "FP": "5 [car, car, car, car, car]",
            "FN": "0 []",
        },
    }


def test_perception_fail_tp_normal(
    create_tp_normal: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
) -> None:
    evaluation_item: Perception = create_tp_normal
    result: PerceptionFrameResult = create_frame_result
    tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(10)
    ]
    result.pass_fail_result.tp_object_results = tp_objects_results
    result.pass_fail_result.fp_object_results = fp_objects_results
    # score 33.3 < NORMAL(50.0)
    frame_dict = evaluation_item.set_frame(result)
    assert evaluation_item.success is False
    assert evaluation_item.summary == "criteria0 (Fail): 94 / 100 -> 94.00%"
    assert frame_dict["PassFail"] == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "TP": "5 [car, car, car, car, car]",
            "FP": "10 [car, car, car, car, car, car, car, car, car, car]",
            "FN": "0 []",
        },
    }


def test_perception_fail_tp_hard(
    create_tp_hard: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
) -> None:
    evaluation_item: Perception = create_tp_hard
    result: PerceptionFrameResult = create_frame_result
    tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    result.pass_fail_result.tp_object_results = tp_objects_results
    result.pass_fail_result.fp_object_results = fp_objects_results
    # score 50.0 < HARD(75.0)
    frame_dict = evaluation_item.set_frame(result)
    assert evaluation_item.success is False
    assert evaluation_item.summary == "criteria0 (Fail): 94 / 100 -> 94.00%"
    assert frame_dict["PassFail"] == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "TP": "5 [car, car, car, car, car]",
            "FP": "5 [car, car, car, car, car]",
            "FN": "0 []",
        },
    }
