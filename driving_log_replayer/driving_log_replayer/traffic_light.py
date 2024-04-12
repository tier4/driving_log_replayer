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
from typing import Any
from typing import Literal

import lanelet2  # noqa
from lanelet2_extension_python.utility.query import getLaneletsWithinRange
from perception_eval.evaluation import PerceptionFrameResult
from pydantic import BaseModel
from pydantic import field_validator
import simplejson as json

from driving_log_replayer.criteria import PerceptionCriteria
from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import number
from driving_log_replayer.scenario import Scenario


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
            s_upper = 202.0

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
    def __post_init__(self) -> None:
        self.condition: Conditions
        self.criteria: PerceptionCriteria = PerceptionCriteria(
            methods=self.condition.CriteriaMethod,
            levels=self.condition.CriteriaLevel,
            distance_range=self.condition.Filter.Distance,
        )

    def set_frame(self, frame: PerceptionFrameResult, map_to_baselink) -> dict:
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
                "Info": {
                    "TP": len(ret_frame.pass_fail_result.tp_object_results),
                    "FP": len(ret_frame.pass_fail_result.fp_object_results),
                    "FN": len(ret_frame.pass_fail_result.fn_objects),
                },
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
        traffic_light_lanelet: Any,
    ) -> None:
        self._frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        for criterion in self.__perception_criterion:
            self._frame[criterion.name] = criterion.set_frame(
                frame,
                map_to_baselink["transform"]["translation"],
                traffic_light_lanelet,  # all traffic_light
            )
        self.update()

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
