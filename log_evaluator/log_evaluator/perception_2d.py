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
from typing import Literal

from perception_eval.evaluation import PerceptionFrameResult
from pydantic import BaseModel

from log_evaluator.criteria import PerceptionCriteria
from log_evaluator.result import EvaluationItem
from log_evaluator.result import ResultBase
from log_evaluator.scenario import number
from log_evaluator.scenario import Scenario


class Conditions(BaseModel):
    PassRate: number
    CriteriaMethod: Literal["num_tp", "num_gt_tp", "label", "metrics_score"] | None = None
    CriteriaLevel: Literal["perfect", "hard", "normal", "easy"] | number | None = None
    TargetCameras: dict[str, int]


class Evaluation(BaseModel):
    UseCaseName: Literal["perception_2d"]
    UseCaseFormatVersion: Literal["0.2.0", "0.3.0"]
    Datasets: list[dict]
    Conditions: Conditions
    PerceptionEvaluationConfig: dict
    CriticalObjectFilterConfig: dict
    PerceptionPassFailConfig: dict


class Perception2DScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Perception(EvaluationItem):
    success: bool = True

    def __post_init__(self) -> None:
        self.condition: Conditions
        self.criteria: PerceptionCriteria = PerceptionCriteria(
            methods=self.condition.CriteriaMethod,
            levels=self.condition.CriteriaLevel,
        )

    def set_frame(
        self,
        frame: PerceptionFrameResult,
    ) -> dict:
        frame_success = "Fail"
        result, _ = self.criteria.get_result(frame)

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
                "Result": {
                    "Total": self.success_str(),
                    "Frame": frame_success,
                },
                "Info": {
                    "TP": len(frame.pass_fail_result.tp_object_results),
                    "FP": len(frame.pass_fail_result.fp_object_results),
                    "FN": len(frame.pass_fail_result.fn_objects),
                },
            },
        }


class Perception2DResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__cameras: dict[str, Perception] = {}
        for camera_type in condition.TargetCameras:
            self.__cameras[camera_type] = Perception(name=camera_type, condition=condition)

    def update(self) -> None:
        tmp_success = True
        tmp_summary = ""
        for v in self.__cameras.values():
            tmp_summary += " " + v.summary
            if not v.success:
                tmp_success = False
        prefix_str = "Passed: " if tmp_success else "Failed: "
        self._success = tmp_success
        self._summary = prefix_str + tmp_summary

    def set_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        map_to_baselink: dict,
        camera_type: str,
    ) -> None:
        self._frame = {
            "CameraType": camera_type,
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        self._frame |= self.__cameras[camera_type].set_frame(frame)
        self.update()

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
