# Copyright (c) 2024 TierIV.inc
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

from pydantic import BaseModel
from typing_extensions import Literal

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario
from driving_log_replayer.scenario import number
from driving_log_replayer_msgs.msg import GroundSegmentationEvalResult


class Condition(BaseModel):
    accuracy_min: number
    accuracy_max: number
    PassRate: number


class Evaluation(BaseModel):
    UseCaseName: Literal["ground_segmentation"]
    UseCaseFormatVersion: Literal["0.3.0"]
    Conditions: Condition


class GroundSegmentationScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class GroundSegmentation(EvaluationItem):
    name: str = "Ground Segmentation"

    def set_frame(self, msg: GroundSegmentationEvalResult) -> dict:
        self.condition : Condition
        self.total += 1

        frame_success = self.condition.accuracy_min <= msg.accuracy <= self.condition.accuracy_max

        if frame_success:
            self.passed += 1

        current_rate = self.rate()
        self.success = current_rate >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {current_rate:.2f}"

        return {
            "GroundSegmentation": {
                "Result": {
                    "Total": self.success_str(),
                    "Frame": "Success" if frame_success else "Fail",
                },
                "Info": {
                    "TP": msg.tp,
                    "FP": msg.fp,
                    "TN": msg.tn,
                    "FN": msg.fn,
                    "Accuracy": msg.accuracy,
                    "Precision": msg.precision,
                    "Recall": msg.recall,
                    "Specificity": msg.specificity,
                    "F1-score": msg.f1_score,
                },
            }
        }


class GroundSegmentationResult(ResultBase):
    def __init__(self, condition: Condition) -> None:
        super().__init__()
        self.__ground_segmentation = GroundSegmentation(condition=condition)

    def update(self) -> None:
        summary_str = f"{self.__ground_segmentation.summary}"
        if self.__ground_segmentation.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(self, msg: GroundSegmentationEvalResult) -> None:
        self._frame = self.__ground_segmentation.set_frame(msg)
        self.update()
