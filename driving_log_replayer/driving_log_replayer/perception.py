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
from typing import List

import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.criteria import PerceptionCriteria
from driving_log_replayer.result import EvaluationItem, ResultBase
from driving_log_replayer.scenario import Scenario, number
from perception_eval.evaluation import PerceptionFrameResult
from pydantic import BaseModel
from std_msgs.msg import ColorRGBA, Header
from typing_extensions import Literal
from visualization_msgs.msg import MarkerArray


class Conditions(BaseModel):
    PassRate: number
    CriteriaMethod: Literal["num_tp", "metrics_score", "metrics_score_maph"] | List[str] | None = None
    CriteriaLevel: Literal["perfect", "hard", "normal", "easy"] | number | None = None


class Evaluation(BaseModel):
    UseCaseName: Literal["perception"]
    UseCaseFormatVersion: Literal["0.5.0", "0.6.0"]
    Datasets: list[dict]
    Conditions: Conditions
    PerceptionEvaluationConfig: dict
    CriticalObjectFilterConfig: dict
    PerceptionPassFailConfig: dict


class PerceptionScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Perception(EvaluationItem):
    name: str = "Perception"

    def __post_init__(self) -> None:
        self.criteria: PerceptionCriteria = PerceptionCriteria(
            methods=self.condition.CriteriaMethod,
            level=self.condition.CriteriaLevel,
        )

    def set_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        header: Header,
        map_to_baselink: dict,
    ) -> tuple[dict, MarkerArray, MarkerArray]:
        self.total += 1
        frame_success = "Fail"
        result = self.criteria.get_result(frame)

        if result.is_success():
            self.passed += 1
            frame_success = "Success"

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

        self.success = self.rate() >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"

        return (
            {
                "Ego": {"TransformStamped": map_to_baselink},
                "FrameName": frame.frame_name,
                "FrameSkip": skip,
                "PassFail": {
                    "Result": {"Total": self.success_str(), "Frame": frame_success},
                    "Info": {
                        "TP": len(frame.pass_fail_result.tp_object_results),
                        "FP": len(frame.pass_fail_result.fp_object_results),
                        "FN": len(frame.pass_fail_result.fn_objects),
                    },
                },
            },
            marker_ground_truth,
            marker_results,
        )


class PerceptionResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__perception = Perception(condition=condition)

    def update(self) -> None:
        summary_str = f"{self.__perception.summary}"
        if self.__perception.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        header: Header,
        map_to_baselink: dict,
    ) -> tuple[MarkerArray, MarkerArray]:
        self._frame, marker_ground_truth, marker_results = self.__perception.set_frame(
            frame,
            skip,
            header,
            map_to_baselink,
        )
        self.update()
        return marker_ground_truth, marker_results

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
