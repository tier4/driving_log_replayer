from dataclasses import dataclass

from pydantic import BaseModel
from typing_extensions import Literal

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario
from driving_log_replayer_msgs.msg import GroundSegmentationEvalResult


class Evaluation(BaseModel):
    UseCaseName: Literal["ground_segmentation"]
    UseCaseFormatVersion: Literal["0.3.0"]


class GroundSegmentationScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Detection(EvaluationItem):
    name: str = "Ground Segmentation"

    def set_frame(self, msg: GroundSegmentationEvalResult) -> dict:
        self.success = (msg.fp == 0) and (msg.fn == 0)
        self.summary = f"{self.name} ({self.success_str()})"

        return {
            "Detection": {
                "Result": {
                    "Total": self.success_str(),
                    "Frame": self.success_str(),
                },
                "Info": {
                    "TP": msg.tp,
                    "FP": msg.fp,
                    "TN": msg.tn,
                    "FN": msg.fn,
                    "Recall": msg.recall,
                    "Specificity": msg.specificity,
                    "F1-score": msg.f1_score,
                    "ProcessTime": msg.process_time,
                },
            }
        }


class GroundSegmentationResult(ResultBase):
    def __init__(self, condition) -> None:
        super().__init__()
        self.__detection = Detection()

    def update(self) -> None:
        summary_str = f"{self.__detection.summary}"
        if self.__detection.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(self, msg: GroundSegmentationEvalResult) -> None:
        self._frame = self.__detection.set_frame(msg)
        self.update()
