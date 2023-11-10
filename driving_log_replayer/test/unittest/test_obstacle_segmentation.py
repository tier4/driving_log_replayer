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

import numpy as np
from perception_eval.common import DynamicObject
from perception_eval.common.label import AutowareLabel
from perception_eval.common.label import Label
from perception_eval.common.schema import FrameID
from perception_eval.common.shape import Shape
from perception_eval.common.shape import ShapeType
from perception_eval.evaluation import DynamicObjectWithSensingResult
from perception_eval.evaluation import SensingFrameResult
from perception_eval.evaluation.sensing.sensing_frame_config import SensingFrameConfig
from pyquaternion import Quaternion
import pytest
from std_msgs.msg import Header

from driving_log_replayer.obstacle_segmentation import Detection
from driving_log_replayer.obstacle_segmentation import NonDetection


@pytest.fixture()
def create_frame_result() -> SensingFrameResult:
    return SensingFrameResult(
        sensing_frame_config=SensingFrameConfig(
            target_uuids=None,
            box_scale_0m=1.0,
            box_scale_100m=1.0,
            min_points_threshold=1,
        ),
        unix_time=123,
        frame_name="12",
    )


@pytest.fixture()
def create_detection() -> Detection:
    return Detection(
        condition={"PassRate": 95.0, "BoundingBoxConfig": None},
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_non_detection() -> NonDetection:
    return NonDetection(
        condition={
            "PassRate": 95.0,
            "ProposedArea": {
                "polygon_2d": [[10.0, 1.5], [10.0, -1.5], [0.0, -1.5], [0.0, 1.5]],
                "z_min": 0.0,
                "z_max": 1.5,
            },
        },
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_dynamic_object() -> DynamicObjectWithSensingResult:
    dynamic_obj = DynamicObject(
        unix_time=123,
        frame_id=FrameID.BASE_LINK,
        position=(0.0, 0.0, 0.0),
        orientation=Quaternion(),
        shape=Shape(ShapeType.BOUNDING_BOX, (3.0, 3.0, 3.0)),
        velocity=(1.0, 2.0, 3.0),
        semantic_score=0.5,
        semantic_label=Label(AutowareLabel.CAR, "12"),
        pointcloud_num=1,
        uuid="dcb2b352232fff50c4fad23718f31611",
    )
    return DynamicObjectWithSensingResult(dynamic_obj, np.ones(shape=(1, 4)), 1.0, 1)


def test_detection_fail_has_no_object(
    create_detection: Callable,
    create_frame_result: Callable,
) -> None:
    evaluation_item: Detection = create_detection
    result: SensingFrameResult = create_frame_result
    # add no detection_success_results, detection_fail_results
    frame_dict, _, _, _ = evaluation_item.set_frame(result, header=Header(), topic_rate=True)
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Detection (Fail): 94 / 100 -> 94.00% (Warn: 0)"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {},
    }


def test_detection_warn(
    create_detection: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
) -> None:
    evaluation_item: Detection = create_detection
    result: SensingFrameResult = create_frame_result
    result.detection_warning_results: list[DynamicObjectWithSensingResult] = [create_dynamic_object]
    frame_dict, _, _, _ = evaluation_item.set_frame(
        result,
        header=Header(),
        topic_rate=True,
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Detection (Fail): 94 / 100 -> 94.00% (Warn: 1)"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Warn"},
        "Info": {
            "DetectionWarn": {
                "PointCloud": {
                    "NumPoints": 1,
                    "Nearest": [1.0, 1.0, 1.0],
                    "Stamp": {"sec": 0, "nanosec": 0},
                },
            },
        },
    }
