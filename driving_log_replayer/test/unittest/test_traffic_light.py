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

from typing import Callable

from perception_eval.common import DynamicObject2D
from perception_eval.common.dataset import FrameGroundTruth
from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.common.label import Label
from perception_eval.common.label import TrafficLightLabel
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation import DynamicObjectWithPerceptionResult
from perception_eval.evaluation import PerceptionFrameResult
from perception_eval.evaluation.metrics import MetricsScoreConfig
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.evaluation.result.perception_pass_fail_result import PassFailResult
import pytest

from driving_log_replayer.traffic_light import Perception


@pytest.fixture()
def create_frame_result() -> PerceptionFrameResult:
    evaluation_config_dict = {
        "evaluation_task": "classification2d",
        "target_labels": ["green", "red", "yellow", "unknown"],
        "allow_matching_unknown": True,
        "merge_similar_labels": False,
        "label_prefix": "traffic_light",
    }
    target_labels = ["green", "red", "yellow", "unknown"]
    m_params: dict = {
        "target_labels": target_labels,
        "center_distance_thresholds": evaluation_config_dict.get("center_distance_thresholds"),
        "plane_distance_thresholds": evaluation_config_dict.get("plane_distance_thresholds"),
        "iou_2d_thresholds": evaluation_config_dict.get("iou_2d_thresholds"),
        "iou_3d_thresholds": evaluation_config_dict.get("iou_3d_thresholds"),
    }
    evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
        dataset_paths=["/tmp/dlr"],  # noqa
        frame_id="cam_traffic_light_near",
        result_root_directory="/tmp/dlr/result/{TIME}",  # noqa
        evaluation_config_dict=evaluation_config_dict,
        load_raw_data=False,
    )

    return PerceptionFrameResult(
        object_results=[],
        frame_ground_truth=FrameGroundTruth(123, "12", []),
        metrics_config=MetricsScoreConfig(
            EvaluationTask.CLASSIFICATION2D,
            **m_params,
        ),
        critical_object_filter_config=CriticalObjectFilterConfig(
            evaluation_config,
            target_labels,
        ),
        frame_pass_fail_config=PerceptionPassFailConfig(evaluation_config, target_labels),
        unix_time=123,
        target_labels=[TrafficLightLabel.GREEN],
    )


@pytest.fixture()
def create_perception() -> Perception:
    return Perception(
        condition={"PassRate": 95.0, "CriteriaMethod": "num_tp", "CriteriaLevel": "easy"},
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_dynamic_object() -> DynamicObjectWithPerceptionResult:
    dynamic_obj_2d = DynamicObject2D(123, "12", 0.5, Label(TrafficLightLabel.GREEN, "12"))
    return DynamicObjectWithPerceptionResult(dynamic_obj_2d, None)


def test_perception_fail(create_perception: Callable, create_frame_result: Callable) -> None:
    evaluation_item: Perception = create_perception
    result: PerceptionFrameResult = create_frame_result
    frame_dict = evaluation_item.set_frame(result, skip=3, map_to_baselink={})
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Traffic Light Perception (Fail): 94 / 100 -> 94.00%"
    assert frame_dict == {
        "Ego": {"TransformStamped": {}},
        "FrameName": "12",
        "FrameSkip": 3,
        "PassFail": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {
                "TP": 0,
                "FP": 0,
                "FN": 0,
            },
        },
    }


def test_perception_success(
    create_perception: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
) -> None:
    evaluation_item: Perception = create_perception
    result: PerceptionFrameResult = create_frame_result
    tp_objects_results: list[DynamicObjectWithPerceptionResult] = [create_dynamic_object]
    result.pass_fail_result.tp_object_results = tp_objects_results
    frame_dict = evaluation_item.set_frame(result, skip=3, map_to_baselink={})
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Traffic Light Perception (Success): 95 / 100 -> 95.00%"
    assert frame_dict == {
        "Ego": {"TransformStamped": {}},
        "FrameName": "12",
        "FrameSkip": 3,
        "PassFail": {
            "Result": {"Total": "Success", "Frame": "Success"},
            "Info": {
                "TP": 1,
                "FP": 0,
                "FN": 0,
            },
        },
    }
