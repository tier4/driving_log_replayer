from dataclasses import dataclass

from pydantic import BaseModel
from typing_extensions import Literal

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import Scenario


class Evaluation(BaseModel):
    UseCaseName: Literal["ground_segmentation"]
    UseCaseFormatVersion: Literal["0.3.0"]


class GroundSegmentationScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Detection(EvaluationItem):
    name: str = "Ground Segmentation"

    def set_frame(self):
        return {}


class GroundSegmentationResult(ResultBase):
    def __init__(self, condition) -> None:
        super().__init__()
        self.__detection = Detection()

    def update(self) -> None:
        pass

    def set_frame(self) -> None:
        self._frame = self.__detection.set_frame()
        self.update()
