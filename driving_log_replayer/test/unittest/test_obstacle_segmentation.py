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
from math import pi

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
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
from pydantic import ValidationError
from pyquaternion import Quaternion as PyQuaternion
import pytest
from shapely.geometry import Polygon
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

from driving_log_replayer.obstacle_segmentation import Detection
from driving_log_replayer.obstacle_segmentation import DetectionCondition
from driving_log_replayer.obstacle_segmentation import get_non_detection_area_in_base_link
from driving_log_replayer.obstacle_segmentation import NonDetection
from driving_log_replayer.obstacle_segmentation import NonDetectionCondition
from driving_log_replayer.obstacle_segmentation import ObstacleSegmentationScenario
from driving_log_replayer.obstacle_segmentation import ProposedAreaCondition
from driving_log_replayer.obstacle_segmentation import transform_proposed_area
from driving_log_replayer.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: ObstacleSegmentationScenario = load_sample_scenario(
        "obstacle_segmentation",
        ObstacleSegmentationScenario,
    )
    assert (
        scenario.Evaluation.Conditions.Detection.BoundingBoxConfig[0][
            "dcb2b352232fff50c4fad23718f31611"
        ].Start
        is None
    )


def test_polygon_clockwise_ok() -> None:
    ProposedAreaCondition(
        polygon_2d=[[10.0, 1.5], [10.0, -1.5], [0.0, -1.5], [0.0, 1.5]],
        z_min=0.0,
        z_max=1.5,
    )


def test_polygon_clockwise_ng() -> None:
    with pytest.raises(ValidationError):
        ProposedAreaCondition(
            polygon_2d=[[10.0, 1.5], [0.0, 1.5], [0.0, -1.5], [10.0, -1.5]],
            z_min=0.0,
            z_max=1.5,
        )


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
        condition=DetectionCondition(PassRate=95.0, BoundingBoxConfig=None),
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_non_detection() -> NonDetection:
    return NonDetection(
        condition=NonDetectionCondition(
            PassRate=95.0,
            ProposedArea=ProposedAreaCondition(
                polygon_2d=[[10.0, 1.5], [10.0, -1.5], [0.0, -1.5], [0.0, 1.5]],
                z_min=0.0,
                z_max=1.5,
            ),
        ),
        total=99,
        passed=94,
    )


@pytest.fixture()
def create_dynamic_object() -> DynamicObjectWithSensingResult:
    dynamic_obj = DynamicObject(
        unix_time=123,
        frame_id=FrameID.BASE_LINK,
        position=(0.0, 0.0, 0.0),
        orientation=PyQuaternion(),
        shape=Shape(ShapeType.BOUNDING_BOX, (10.0, 10.0, 10.0)),
        velocity=(1.0, 2.0, 3.0),
        semantic_score=0.5,
        semantic_label=Label(AutowareLabel.CAR, "12"),
        pointcloud_num=1,
        uuid="dcb2b352232fff50c4fad23718f31611",
    )
    pointcloud = np.array([[1.0, 1.0, 1.0, 0.5], [1.2, 1.2, 1.2, 0.5]])
    return DynamicObjectWithSensingResult(dynamic_obj, pointcloud, 1.0, 1)


@pytest.fixture()
def create_distance_dict() -> dict:
    return {
        "0-1": 0,
        "1-2": 1,
        "2-3": 1,
        "3-4": 0,
        "4-5": 0,
        "5-6": 0,
        "6-7": 0,
        "7-8": 0,
        "8-9": 0,
        "9-10": 0,
        "10-11": 0,
        "11-12": 0,
        "12-13": 0,
        "13-14": 0,
        "14-15": 0,
        "15-16": 0,
        "16-17": 0,
        "17-18": 0,
        "18-19": 0,
        "19-20": 0,
        "20-21": 0,
        "21-22": 0,
        "22-23": 0,
        "23-24": 0,
        "24-25": 0,
        "25-26": 0,
        "26-27": 0,
        "27-28": 0,
        "28-29": 0,
        "29-30": 0,
        "30-31": 0,
        "31-32": 0,
        "32-33": 0,
        "33-34": 0,
        "34-35": 0,
        "35-36": 0,
        "36-37": 0,
        "37-38": 0,
        "38-39": 0,
        "39-40": 0,
        "40-41": 0,
        "41-42": 0,
        "42-43": 0,
        "43-44": 0,
        "44-45": 0,
        "45-46": 0,
        "46-47": 0,
        "47-48": 0,
        "48-49": 0,
        "49-50": 0,
        "50-51": 0,
        "51-52": 0,
        "52-53": 0,
        "53-54": 0,
        "54-55": 0,
        "55-56": 0,
        "56-57": 0,
        "57-58": 0,
        "58-59": 0,
        "59-60": 0,
        "60-61": 0,
        "61-62": 0,
        "62-63": 0,
        "63-64": 0,
        "64-65": 0,
        "65-66": 0,
        "66-67": 0,
        "67-68": 0,
        "68-69": 0,
        "69-70": 0,
        "70-71": 0,
        "71-72": 0,
        "72-73": 0,
        "73-74": 0,
        "74-75": 0,
        "75-76": 0,
        "76-77": 0,
        "77-78": 0,
        "78-79": 0,
        "79-80": 0,
        "80-81": 0,
        "81-82": 0,
        "82-83": 0,
        "83-84": 0,
        "84-85": 0,
        "85-86": 0,
        "86-87": 0,
        "87-88": 0,
        "88-89": 0,
        "89-90": 0,
        "90-91": 0,
        "91-92": 0,
        "92-93": 0,
        "93-94": 0,
        "94-95": 0,
        "95-96": 0,
        "96-97": 0,
        "97-98": 0,
        "98-99": 0,
        "99-100": 0,
    }


@pytest.fixture()
def create_annotation_dict() -> dict:
    return {
        "Scale": {
            "x": 10.0,
            "y": 10.0,
            "z": 10.0,
        },
        "Position": {
            "position": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0,
            },
        },
        "UUID": "dcb2b352232fff50c4fad23718f31611",
        "StampFloat": 0.000123,
    }


def test_detection_fail_has_no_object(
    create_detection: Callable,
    create_frame_result: Callable,
) -> None:
    evaluation_item: Detection = create_detection
    result: SensingFrameResult = create_frame_result
    # add no detection_success_results, detection_fail_results
    frame_dict, _, _, _ = evaluation_item.set_frame(
        result,
        header=Header(),
        topic_rate=True,
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Detection (Fail): 94 / 100 -> 94.00% (Warn: 0)"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {},
    }


def test_detection_invalid(
    create_frame_result: Callable,
) -> None:
    evaluation_item: Detection = Detection(condition=None)
    result: SensingFrameResult = create_frame_result
    frame_dict, _, _, _ = evaluation_item.set_frame(
        result,
        header=Header(),
        topic_rate=True,
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Invalid"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Invalid"},
        "Info": {},
    }


def test_detection_warn(
    create_detection: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
    create_annotation_dict: Callable,
) -> None:
    evaluation_item: Detection = create_detection
    result: SensingFrameResult = create_frame_result
    annotation_dict: dict = create_annotation_dict
    result.detection_warning_results: list[DynamicObjectWithSensingResult] = [
        create_dynamic_object,
    ]
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
                "Annotation": annotation_dict,
                "PointCloud": {
                    "NumPoints": 2,
                    "Nearest": [1.0, 1.0, 1.0],
                    "Stamp": {"sec": 0, "nanosec": 0},
                },
            },
        },
    }


def test_detection_success(
    create_detection: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
    create_annotation_dict: Callable,
) -> None:
    evaluation_item: Detection = create_detection
    result: SensingFrameResult = create_frame_result
    annotation_dict: dict = create_annotation_dict
    result.detection_success_results: list[DynamicObjectWithSensingResult] = [
        create_dynamic_object,
    ]
    frame_dict, _, _, _ = evaluation_item.set_frame(
        result,
        header=Header(),
        topic_rate=True,
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Detection (Success): 95 / 100 -> 95.00% (Warn: 0)"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "DetectionSuccess": {
                "Annotation": annotation_dict,
                "PointCloud": {
                    "NumPoints": 2,
                    "Nearest": [1.0, 1.0, 1.0],
                    "Stamp": {"sec": 0, "nanosec": 0},
                },
            },
        },
    }


def test_detection_topic_rate_fail(
    create_detection: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
    create_annotation_dict: Callable,
) -> None:
    evaluation_item: Detection = create_detection
    result: SensingFrameResult = create_frame_result
    annotation_dict: dict = create_annotation_dict
    result.detection_success_results: list[DynamicObjectWithSensingResult] = [
        create_dynamic_object,
    ]
    frame_dict, _, _, _ = evaluation_item.set_frame(
        result,
        header=Header(),
        topic_rate=False,  # false
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Detection (Fail): 94 / 100 -> 94.00% (Warn: 0)"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "DetectionSuccess": {
                "Annotation": annotation_dict,
                "PointCloud": {
                    "NumPoints": 2,
                    "Nearest": [1.0, 1.0, 1.0],
                    "Stamp": {"sec": 0, "nanosec": 0},
                },
            },
        },
    }


def test_detection_fail(
    create_detection: Callable,
    create_frame_result: Callable,
    create_dynamic_object: Callable,
    create_annotation_dict: Callable,
) -> None:
    evaluation_item: Detection = create_detection
    result: SensingFrameResult = create_frame_result
    annotation_dict: dict = create_annotation_dict
    result.detection_success_results: list[DynamicObjectWithSensingResult] = [
        create_dynamic_object,
    ]
    result.detection_fail_results: list[DynamicObjectWithSensingResult] = [
        create_dynamic_object,
    ]
    frame_dict, _, _, _ = evaluation_item.set_frame(
        result,
        header=Header(),
        topic_rate=True,
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Detection (Fail): 94 / 100 -> 94.00% (Warn: 0)"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "DetectionSuccess": {
                "Annotation": annotation_dict,
                "PointCloud": {
                    "NumPoints": 2,
                    "Nearest": [1.0, 1.0, 1.0],
                    "Stamp": {"sec": 0, "nanosec": 0},
                },
            },
            "DetectionFail": {
                "Annotation": annotation_dict,
                "PointCloud": {
                    "NumPoints": 2,
                    "Nearest": [1.0, 1.0, 1.0],
                    "Stamp": {"sec": 0, "nanosec": 0},
                },
            },
        },
    }


def test_non_detection_invalid() -> None:
    evaluation_item: Detection = NonDetection(condition=None)
    pointcloud = np.array([[1.0, 1.0, 1.0, 0.5], [1.2, 1.2, 1.2, 0.5]])
    frame_dict, _, _ = evaluation_item.set_frame(
        [pointcloud],
        header=Header(),
        topic_rate=True,
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Invalid"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Invalid"},
        "Info": {},
    }


def test_non_detection_fail(
    create_non_detection: Callable,
    create_distance_dict: Callable,
) -> None:
    evaluation_item: NonDetection = create_non_detection
    pointcloud = np.array([[1.0, 1.0, 1.0, 0.5], [1.2, 1.2, 1.2, 0.5]])
    frame_dict, _, _ = evaluation_item.set_frame(
        [pointcloud],
        header=Header(),
        topic_rate=True,
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "NonDetection (Fail): 94 / 100 -> 94.00%"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "PointCloud": {
                "NumPoints": 2,
                "Distance": create_distance_dict,
            },
        },
    }


def test_non_detection_topic_rate_fail(
    create_non_detection: Callable,
) -> None:
    evaluation_item: NonDetection = create_non_detection
    # no pointcloud
    frame_dict, _, _ = evaluation_item.set_frame(
        [],
        header=Header(),
        topic_rate=False,
    )  # topic rate error
    assert evaluation_item.success is False
    assert evaluation_item.summary == "NonDetection (Fail): 94 / 100 -> 94.00%"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {},
    }


def test_non_detection_success(
    create_non_detection: Callable,
) -> None:
    evaluation_item: NonDetection = create_non_detection
    # no pointcloud
    frame_dict, _, _ = evaluation_item.set_frame([], header=Header(), topic_rate=True)
    assert evaluation_item.success is True
    assert evaluation_item.summary == "NonDetection (Success): 95 / 100 -> 95.00%"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {},
    }


def test_transform_proposed_area() -> None:
    header_base_link = Header(frame_id="base_link")
    header_map = Header(frame_id="map")
    q = quaternion_from_euler(0.0, 0.0, -pi / 2)
    map_to_baselink = TransformStamped(
        header=header_map,
        child_frame_id="base_link",
        transform=Transform(
            translation=Vector3(x=10.0, y=10.0, z=0.0),
            rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        ),
    )
    proposed_area = ProposedAreaCondition(
        polygon_2d=[[2.0, 2.0], [0.0, 0.0], [-2.0, 2.0]],
        z_min=0.0,
        z_max=2.0,
    )
    proposed_area_in_map, z = transform_proposed_area(
        proposed_area,
        header_base_link,
        map_to_baselink,
    )
    assert proposed_area_in_map == Polygon(
        ((12.0, 8.0), (10.0, 10.0), (12.0, 12.0)),
    )  # do not set z. Polygon and Polygon Z is different.
    assert z == 0.0  # noqa


def test_get_non_detection_area_in_base_link() -> None:
    header_base_link = Header(frame_id="base_link")
    intersection_polygon = Polygon(((12.0, 8.0), (10.0, 10.0), (12.0, 12.0)))
    base_link_to_map = TransformStamped(
        header=header_base_link,
        child_frame_id="map",
        transform=Transform(
            translation=Vector3(x=10.0, y=-10.000000000000002, z=0.0),
            rotation=Quaternion(
                x=0.0,
                y=0.0,
                z=0.7071067811865475,
                w=0.7071067811865476,
            ),
        ),
    )
    ans_non_detection_list = [
        [2.0000000000000027, 2.0, 0.0],
        [1.7763568394002505e-15, 0.0, 0.0],
        [-1.9999999999999964, 2.0000000000000018, 0.0],
        [2.0000000000000027, 2.0, 0.0],
        [2.0000000000000027, 2.0, 2.0],
        [1.7763568394002505e-15, 0.0, 2.0],
        [-1.9999999999999964, 2.0000000000000018, 2.0],
        [2.0000000000000027, 2.0, 2.0],
    ]
    line_strip, non_detection_list = get_non_detection_area_in_base_link(
        intersection_polygon,
        header_base_link,
        0.0,
        2.0,
        0.0,
        base_link_to_map,
        1,
    )
    ans_line_strip = Marker(
        header=header_base_link,
        ns="intersection",
        id=1,
        type=Marker.LINE_STRIP,
        action=Marker.ADD,
        color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3),
        scale=Vector3(x=0.2, y=0.2, z=0.2),
        lifetime=Duration(nanosec=200_000_000),
    )
    assert non_detection_list == ans_non_detection_list
    for point in ans_non_detection_list:
        ans_line_strip.points.append(Point(x=point[0], y=point[1], z=point[2]))
    assert line_strip == ans_line_strip
