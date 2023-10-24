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
from typing import ClassVar

from perception_eval.evaluation import PerceptionFrameResult

from driving_log_replayer.criteria import PerceptionCriteria
from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase


@dataclass
class Perception(EvaluationItem):
    name: ClassVar[str] = "Perception"

    def __post_init__(self) -> None:
        self.criteria: PerceptionCriteria = PerceptionCriteria(
            method=self.condition.get("CriteriaMethod"),
            level=self.condition.get("CriteriaLevel"),
        )

    def set_frame(self, frame: PerceptionFrameResult, skip: int, map_to_baselink: dict) -> dict:
        self.total += 1
        frame_success = "Fail"
        result = self.criteria.get_result(frame)

        if result.is_success():
            self.passed += 1
            frame_success = "Success"

        self.success = self.rate() >= self.condition["PassRate"]
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"

        return {
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
        }


class TrafficLightResult(ResultBase):
    def __init__(self, condition: dict) -> None:
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
        self._frame = self.__perception.set_frame(frame, skip, map_to_baselink)
        self.update()

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
