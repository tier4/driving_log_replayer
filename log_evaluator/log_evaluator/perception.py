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
import sys
from typing import Literal

from perception_eval.evaluation import PerceptionFrameResult
from pydantic import BaseModel
from pydantic import field_validator
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from log_evaluator.criteria import PerceptionCriteria
import log_evaluator.perception_eval_conversions as eval_conversions
from log_evaluator.perception_eval_conversions import FrameDescriptionWriter
from log_evaluator.perception_eval_conversions import summarize_pass_fail_result
from log_evaluator.result import EvaluationItem
from log_evaluator.result import ResultBase
from log_evaluator.scenario import number
from log_evaluator.scenario import Scenario


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
        Literal[
            "num_tp",
            "num_gt_tp",
            "label",
            "metrics_score",
            "metrics_score_maph",
            "velocity_x_error",
            "velocity_y_error",
            "speed_error",
            "yaw_error",
        ]
        | list[str]
        | None
    ) = None
    CriteriaLevel: (
        Literal["perfect", "hard", "normal", "easy"] | list[str] | number | list[number] | None
    ) = None
    Filter: Filter


class Conditions(BaseModel):
    Criterion: list[Criteria]


class Evaluation(BaseModel):
    UseCaseName: Literal["perception"]
    UseCaseFormatVersion: Literal["1.0.0"]
    Datasets: list[dict]
    Conditions: Conditions
    PerceptionEvaluationConfig: dict
    CriticalObjectFilterConfig: dict
    PerceptionPassFailConfig: dict


class PerceptionScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Perception(EvaluationItem):
    success: bool = True

    def __post_init__(self) -> None:
        self.criteria = PerceptionCriteria(
            methods=self.condition.CriteriaMethod,
            levels=self.condition.CriteriaLevel,
            distance_range=self.condition.Filter.Distance,
        )

    def set_frame(self, frame: PerceptionFrameResult) -> dict:
        frame_success = "Fail"
        # ret_frame might be filtered frame result or original frame result.
        result, ret_frame = self.criteria.get_result(frame)

        if result is None:
            self.no_gt_no_obj += 1
            return {"NoGTNoObj": self.no_gt_no_obj}
        if result.is_success():
            self.passed += 1
            frame_success = "Success"

        self.total += 1
        self.success: bool = self.rate() >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"

        return {
            "PassFail": {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": summarize_pass_fail_result(ret_frame.pass_fail_result),
            },
            "Objects": FrameDescriptionWriter.extract_pass_fail_objects_description(
                ret_frame.pass_fail_result,
            ),
        }


class PerceptionResult(ResultBase):
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
        header: Header,
        map_to_baselink: dict,
    ) -> tuple[MarkerArray, MarkerArray]:
        self._frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        for criterion in self.__perception_criterion:
            self._frame[criterion.name] = criterion.set_frame(frame)
        self.update()
        marker_ground_truth, marker_results = self.create_ros_msg(frame, header)
        return marker_ground_truth, marker_results

    def set_warn_frame(self, msg: str, skip: int) -> None:
        self._frame = {
            "Warning": msg,
            "FrameSkip": skip,
        }

    def create_ros_msg(
        self,
        frame: PerceptionFrameResult,
        header: Header,
    ) -> tuple[MarkerArray, MarkerArray]:
        marker_ground_truth = MarkerArray()
        color_success = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)

        for cnt, obj in enumerate(frame.frame_ground_truth.objects, start=1):
            bbox, uuid = eval_conversions.object_state_to_ros_box_and_uuid(
                obj.state,
                header,
                "ground_truth",
                cnt,
                color_success,
                obj.uuid,
            )
            marker_ground_truth.markers.append(bbox)
            marker_ground_truth.markers.append(uuid)

        marker_results = eval_conversions.pass_fail_result_to_ros_points_array(
            frame.pass_fail_result,
            header,
        )
        return marker_ground_truth, marker_results

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
