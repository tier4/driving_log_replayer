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

from perception_eval.common import DynamicObject
from perception_eval.common.dataset import FrameGroundTruth
from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.common.label import AutowareLabel
from perception_eval.common.label import Label
from perception_eval.common.schema import FrameID
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation import DynamicObjectWithPerceptionResult
from perception_eval.evaluation import PerceptionFrameResult
from perception_eval.evaluation.metrics import MetricsScoreConfig
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from pyquaternion import Quaternion
import pytest
from std_msgs.msg import Header

from driving_log_replayer.perception import Perception


@pytest.fixture()
def create_frame_result() -> PerceptionFrameResult:
    target_labels = ["car", "bicycle", "pedestrian", "motorbike"]
    evaluation_config_dict = {
        "evaluation_task": "detection",
        "target_labels": target_labels,
        "max_x_position": 102.4,
        "max_y_position": 102.4,
        "max_matchable_radii": [5.0, 3.0, 3.0, 3.0],
        "merge_similar_labels": False,
        "allow_matching_unknown": True,
        "ignore_attributes": ["cycle_state.without_rider"],
        "center_distance_thresholds": [[1.0, 1.0, 1.0, 1.0], [2.0, 2.0, 2.0, 2.0]],
        "plane_distance_thresholds": [2.0, 30.0],
        "iou_2d_thresholds": [0.5],
        "iou_3d_thresholds": [0.5],
        "min_point_numbers": [0, 0, 0, 0],
        "label_prefix": "autoware",
    }
    m_params: dict = {
        "target_labels": target_labels,
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
            target_labels,
            max_x_position_list=[30.0, 30.0, 30.0, 30.0],
            max_y_position_list=[30.0, 30.0, 30.0, 30.0],
        ),
        frame_pass_fail_config=PerceptionPassFailConfig(evaluation_config, target_labels),
        unix_time=123,
        target_labels=[AutowareLabel.CAR],
    )


@pytest.fixture()
def create_tp_normal() -> Perception:
    return Perception(
        condition={"PassRate": 95.0, "CriteriaMethod": "num_tp", "CriteriaLevel": "normal"},
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_tp_hard() -> Perception:
    return Perception(
        condition={"PassRate": 95.0, "CriteriaMethod": "num_tp", "CriteriaLevel": "hard"},
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_dynamic_object() -> DynamicObjectWithPerceptionResult:
    dynamic_obj = DynamicObject(
        123,
        FrameID.BASE_LINK,
        0.5,
        (1.0, 2.0, 3.0),
        Quaternion(),
        None,
        0.5,
        Label(AutowareLabel.CAR, "12"),
    )
    return DynamicObjectWithPerceptionResult(dynamic_obj, dynamic_obj)


def test_perception_fail_has_no_object(
    create_tp_normal: Callable,
    create_frame_result: Callable,
) -> None:
    evaluation_item: Perception = create_tp_normal
    result: PerceptionFrameResult = create_frame_result
    # add no tp_object_results, fp_object_results
    frame_dict, _, _ = evaluation_item.set_frame(
        result,
        skip=3,
        header=Header(),
        map_to_baselink={},
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Perception (Fail): 94 / 100 -> 94.00%"
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


# def test_perception_success_tp_normal(
#     create_tp_normal: Callable,
#     create_frame_result: Callable,
#     create_dynamic_object: Callable,
# ) -> None:
#     evaluation_item: Perception = create_tp_normal
#     result: PerceptionFrameResult = create_frame_result
#     tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
#         create_dynamic_object for i in range(5)
#     ]
#     fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
#         create_dynamic_object for i in range(5)
#     ]
#     result.pass_fail_result.tp_object_results = tp_objects_results
#     result.pass_fail_result.fp_object_results = fp_objects_results
#     # score 50.0 >= NORMAL(50.0)
#     frame_dict, _, _ = evaluation_item.set_frame(
#         result,
#         skip=3,
#         header=Header(),
#         map_to_baselink={},
#     )
#     assert evaluation_item.success is True
#     assert evaluation_item.summary == "Perception (Success): 95 / 100 -> 95.00%"
#     assert frame_dict == {
#         "Ego": {"TransformStamped": {}},
#         "FrameName": "12",
#         "FrameSkip": 3,
#         "PassFail": {
#             "Result": {"Total": "Success", "Frame": "Success"},
#             "Info": {
#                 "TP": 5,
#                 "FP": 5,
#                 "FN": 0,
#             },
#         },
#     }


# def test_perception_fail_tp_normal(
#     create_tp_normal: Callable,
#     create_frame_result: Callable,
#     create_dynamic_object: Callable,
# ) -> None:
#     evaluation_item: Perception = create_tp_normal
#     result: PerceptionFrameResult = create_frame_result
#     tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
#         create_dynamic_object for i in range(5)
#     ]
#     fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
#         create_dynamic_object for i in range(10)
#     ]
#     result.pass_fail_result.tp_object_results = tp_objects_results
#     result.pass_fail_result.fp_object_results = fp_objects_results
#     # score 33.3 < NORMAL(50.0)
#     frame_dict, _, _ = evaluation_item.set_frame(
#         result,
#         skip=3,
#         header=Header(),
#         map_to_baselink={},
#     )
#     assert evaluation_item.success is False
#     assert evaluation_item.summary == "Perception (Fail): 94 / 100 -> 94.00%"
#     assert frame_dict == {
#         "Ego": {"TransformStamped": {}},
#         "FrameName": "12",
#         "FrameSkip": 3,
#         "PassFail": {
#             "Result": {"Total": "Fail", "Frame": "Fail"},
#             "Info": {
#                 "TP": 5,
#                 "FP": 10,
#                 "FN": 0,
#             },
#         },
#     }


# def test_perception_fail_tp_hard(
#     create_tp_hard: Callable,
#     create_frame_result: Callable,
#     create_dynamic_object: Callable,
# ) -> None:
#     evaluation_item: Perception = create_tp_hard
#     result: PerceptionFrameResult = create_frame_result
#     tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
#         create_dynamic_object for i in range(5)
#     ]
#     fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
#         create_dynamic_object for i in range(5)
#     ]
#     result.pass_fail_result.tp_object_results = tp_objects_results
#     result.pass_fail_result.fp_object_results = fp_objects_results
#     # score 50.0 < HARD(75.0)
#     frame_dict, _, _ = evaluation_item.set_frame(
#         result,
#         skip=3,
#         header=Header(),
#         map_to_baselink={},
#     )
#     assert evaluation_item.success is False
#     assert evaluation_item.summary == "Perception (Fail): 94 / 100 -> 94.00%"
#     assert frame_dict == {
#         "Ego": {"TransformStamped": {}},
#         "FrameName": "12",
#         "FrameSkip": 3,
#         "PassFail": {
#             "Result": {"Total": "Fail", "Frame": "Fail"},
#             "Info": {
#                 "TP": 5,
#                 "FP": 5,
#                 "FN": 0,
#             },
#         },
#     }
