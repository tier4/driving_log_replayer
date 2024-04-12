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
from typing import Literal

from perception_eval.evaluation import PerceptionFrameResult
from pydantic import BaseModel
import simplejson as json

from driving_log_replayer.criteria import PerceptionCriteria
from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import number
from driving_log_replayer.scenario import Scenario


class Conditions(BaseModel):
    PassRate: number
    CriteriaMethod: Literal["num_tp", "metrics_score"] | None = None
    CriteriaLevel: Literal["perfect", "hard", "normal", "easy"] | number | None = None


class Evaluation(BaseModel):
    UseCaseName: Literal["traffic_light"]
    UseCaseFormatVersion: Literal["0.2.0", "0.3.0"]
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
    name: str = "Traffic Light Perception"

    def __post_init__(self) -> None:
        self.condition: Conditions
        self.criteria: PerceptionCriteria = PerceptionCriteria(
            methods=self.condition.CriteriaMethod,
            levels=self.condition.CriteriaLevel,
        )

    def set_frame(self, frame: PerceptionFrameResult) -> dict:
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
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {
                    "TP": len(frame.pass_fail_result.tp_object_results),
                    "FP": len(frame.pass_fail_result.fp_object_results),
                    "FN": len(frame.pass_fail_result.fn_objects),
                },
            },
        }


class TrafficLightResult(ResultBase):
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
        map_to_baselink: dict,
    ) -> None:
        self._frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        self._frame |= self.__perception.set_frame(frame)
        self.update()

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
