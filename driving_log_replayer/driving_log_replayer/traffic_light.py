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

from dataclasses import dataclass
import logging
from pathlib import Path
import sys
from typing import Literal

from autoware_perception_msgs.msg import TrafficSignalElement
from perception_eval.evaluation import PerceptionFrameResult
from pydantic import BaseModel
from pydantic import field_validator
import simplejson as json

from driving_log_replayer.criteria import PerceptionCriteria
from driving_log_replayer.perception_eval_conversions import summarize_pass_fail_result
from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import number
from driving_log_replayer.scenario import Scenario

TRAFFIC_LIGHT_LABEL_MAPPINGS: list[tuple[set, str]] = [
    ({"green"}, "green"),
    ({"green", "straight"}, "green_straight"),
    ({"green", "left"}, "green_left"),
    ({"green", "right"}, "green_right"),
    ({"yellow"}, "yellow"),
    ({"yellow", "straight"}, "yellow_straight"),
    ({"yellow", "left"}, "yellow_left"),
    ({"yellow", "right"}, "yellow_right"),
    ({"yellow", "straight", "left"}, "yellow_straight_left"),
    ({"yellow", "straight", "right"}, "yellow_straight_right"),
    ({"red"}, "red"),
    ({"red", "straight"}, "red_straight"),
    ({"red", "left"}, "red_left"),
    ({"red", "right"}, "red_right"),
    ({"red", "straight", "left"}, "red_straight_left"),
    ({"red", "straight", "right"}, "red_straight_right"),
    ({"red", "straight", "left", "right"}, "red_straight_left_right"),
    ({"red", "right", "diagonal"}, "red_rightdiagonal"),
    ({"red", "left", "diagonal"}, "red_leftdiagonal"),
]


def get_camera_frame_id_from_t4_dataset(dataset_path: str, target_tlr_camera: str) -> str:
    status_path = Path(dataset_path, "status.json")
    with status_path.open() as status_file:
        status_json = json.load(status_file)
        camera_sensors: list[dict] = status_json.get(
            "rosbag2_to_non_annotated_t4_converter",
            {},
        ).get(
            "_camera_sensors",
            {},
        )
        for camera in camera_sensors:
            if camera["channel"] == target_tlr_camera:
                # image_rawじゃない可能性も考慮したほうがいいか。
                # /sensing/camera/camera7/image_raw/compressed -> camera7/camera_link
                camera_name = (
                    camera["topic"]
                    .replace("/sensing/camera/", "")
                    .replace("/image_raw/compressed", "")
                )
    return camera_name + "/camera_link"


def get_traffic_light_label_str(elements: list[TrafficSignalElement]) -> str:  # noqa
    label_infos = []
    for element in elements:
        if element.shape == TrafficSignalElement.CIRCLE:
            if element.color == TrafficSignalElement.RED:
                label_infos.append("red")
            elif element.color == TrafficSignalElement.AMBER:
                label_infos.append("yellow")
            elif element.color == TrafficSignalElement.GREEN:
                label_infos.append("green")
            continue

        if element.shape == TrafficSignalElement.UP_ARROW:
            label_infos.append("straight")
        elif element.shape == TrafficSignalElement.LEFT_ARROW:
            label_infos.append("left")
        elif element.shape == TrafficSignalElement.RIGHT_ARROW:
            label_infos.append("right")
        elif element.shape in (
            TrafficSignalElement.UP_LEFT_ARROW,
            TrafficSignalElement.DOWN_LEFT_ARROW,
        ):
            label_infos.append("left")
            label_infos.append("diagonal")
        elif element.shape in (
            TrafficSignalElement.UP_RIGHT_ARROW,
            TrafficSignalElement.DOWN_RIGHT_ARROW,
        ):
            label_infos.append("right")
            label_infos.append("diagonal")

    label_infos = set(label_infos)

    for info_set, label in TRAFFIC_LIGHT_LABEL_MAPPINGS:
        if label_infos == info_set:
            return label

    return "unknown"


def get_most_probable_element(
    elements: list[TrafficSignalElement],
) -> TrafficSignalElement:
    index: int = elements.index(max(elements, key=lambda x: x.confidence))
    return elements[index]


class Filter(BaseModel):
    Distance: tuple[float, float] | None = None
    # add filter condition here

    @field_validator("Distance", mode="before")
    @classmethod
    def validate_distance_range(cls, v: str | None) -> tuple[number, number] | None:
        if v is None:
            return None

        err_msg = f"{v} is not valid distance range, expected ordering min-max with min < max."

        s_lower, s_upper = v.split("-")
        if s_upper == "":
            s_upper = sys.float_info.max

        lower = float(s_lower)
        upper = float(s_upper)

        if lower >= upper:
            raise ValueError(err_msg)
        return (lower, upper)


class Criteria(BaseModel):
    PassRate: number
    CriteriaMethod: (
        Literal["num_tp", "label", "metrics_score", "metrics_score_maph"] | list[str] | None
    ) = None
    CriteriaLevel: (
        Literal["perfect", "hard", "normal", "easy"] | list[str] | number | list[number] | None
    ) = None
    Filter: Filter


class Conditions(BaseModel):
    Criterion: list[Criteria]


class Evaluation(BaseModel):
    UseCaseName: Literal["traffic_light"]
    UseCaseFormatVersion: Literal["1.0.0"]
    Datasets: list[dict]
    Conditions: Conditions
    PerceptionEvaluationConfig: dict
    CriticalObjectFilterConfig: dict
    PerceptionPassFailConfig: dict


class TrafficLightScenario(Scenario):
    Evaluation: Evaluation


class FailResultHolder:
    def __init__(self, save_dir: str) -> None:
        self.save_path: str = Path(save_dir, "fail_info.json")
        self.buffer = []

    def add_frame(self, frame_result: PerceptionFrameResult) -> None:
        if frame_result.pass_fail_result.get_fail_object_num() <= 0:
            return
        info = {"fp": [], "fn": []}
        info["timestamp"] = frame_result.frame_ground_truth.unix_time
        for fp_result in frame_result.pass_fail_result.fp_object_results:
            est_label = fp_result.estimated_object.semantic_label.label.value
            gt_label = (
                fp_result.ground_truth_object.semantic_label.label.value
                if fp_result.ground_truth_object is not None
                else None
            )
            info["fp"].append({"est": est_label, "gt": gt_label})
        for fn_object in frame_result.pass_fail_result.fn_objects:
            info["fn"].append({"est": None, "gt": fn_object.semantic_label.label.value})

        info_str = f"Fail timestamp: {info}"
        logging.info(info_str)
        self.buffer.append(info)

    def save(self) -> None:
        with self.save_path.open("w") as f:
            json.dump(self.buffer, f, ensure_ascii=False, indent=4)


@dataclass
class Perception(EvaluationItem):
    success: bool = True
    name: str = "Traffic Light Perception"

    def __post_init__(self) -> None:
        self.condition: Conditions
        self.criteria: PerceptionCriteria = PerceptionCriteria(
            methods=self.condition.CriteriaMethod,
            levels=self.condition.CriteriaLevel,
            distance_range=self.condition.Filter.Distance,
        )

    def set_frame(self, frame: PerceptionFrameResult) -> dict:
        frame_success = "Fail"
        result, ret_frame = self.criteria.get_result(frame)

        if result is None:
            self.no_gt_no_obj += 1
            return {"NoGTNoObj": self.no_gt_no_obj}

        if result.is_success():
            self.passed += 1
            frame_success = "Success"

        self.total += 1
        self.success = self.rate() >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"

        return {
            "PassFail": {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": summarize_pass_fail_result(ret_frame.pass_fail_result),
            },
        }


class TrafficLightResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__perception_criterion: list[Perception] = []
        for i, criteria in enumerate(condition.Criterion):
            self.__perception_criterion.append(
                Perception(name=f"criteria{i}", condition=criteria),
            )

    def update(self) -> None:
        all_summary: list[str] = []
        all_success: list[bool] = []
        for criterion in self.__perception_criterion:
            tmp_success = criterion.success
            prefix_str = "Passed: " if tmp_success else "Failed: "
            all_summary.append(prefix_str + criterion.summary)
            all_success.append(tmp_success)
        self._summary = ", ".join(all_summary)
        self._success = all(all_success)

    def set_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        map_to_baselink: dict,
    ) -> None:
        self._frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        for criterion in self.__perception_criterion:
            self._frame[criterion.name] = criterion.set_frame(frame)
        self.update()

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
