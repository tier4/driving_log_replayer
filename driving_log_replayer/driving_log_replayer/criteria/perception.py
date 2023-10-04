from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from numbers import Number
from typing import Optional, Union

from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.evaluation import PerceptionFrameResult


class SuccessFail(Enum):
    SUCCESS = "Success"
    FAIL = "Fail"

    def __str__(self) -> str:
        return self.value

    def is_success(self) -> bool:
        return self == SuccessFail.SUCCESS


class CriteriaLevel(Enum):
    PERFECT = 100.0
    HARD = 90.0
    NORMAL = 75.0
    EASY = 50.0

    def is_ok(self, score: Number) -> bool:
        return score >= self.value

    @classmethod
    def from_str(cls, value: str) -> CriteriaLevel:
        name: str = value.upper()
        return cls.__members__[name]


class CriteriaMode(Enum):
    NUM_FAILED_OBJECT = "num_failed_object"
    METRICS_SCORE = "metrics_score"

    @classmethod
    def from_str(cls, value: str) -> CriteriaMode:
        name: str = value.upper()
        return cls.__members__[name]


class CriteriaMethod(ABC):
    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__()
        self.level: CriteriaLevel = level

    def get_result(self, frame: PerceptionFrameResult) -> SuccessFail:
        if self.has_objects(frame) is False:
            return SuccessFail.FAIL
        score: float = self.calculate_score(frame)
        return SuccessFail.SUCCESS if self.level.is_ok(score) else SuccessFail.FAIL

    @staticmethod
    def has_objects(frame: PerceptionFrameResult) -> bool:
        num_success: int = frame.pass_fail_result.get_num_success()
        num_fail: int = frame.pass_fail_result.get_num_fail()
        return num_success + num_fail > 0

    @staticmethod
    @abstractmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        pass


class NumFailObject(CriteriaMethod):
    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        num_success: int = frame.pass_fail_result.get_num_success()
        num_objects: int = num_success + frame.pass_fail_result.get_num_fail()
        return 100.0 * num_success / num_objects if num_objects != 0 else 0.0


class MetricsScore(CriteriaMethod):
    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        if frame.metrics_score.evaluation_task == EvaluationTask.CLASSIFICATION2D:
            scores = [
                acc.accuracy
                for score in frame.metrics_score.classification_scores
                for acc in score.accuracies
                if acc.accuracy != float("inf")
            ]
        else:
            scores = [map_.map for map_ in frame.metrics_score.maps if map_.map != float("inf")]

        return 100.0 * sum(scores) / len(scores) if len(scores) != 0 else 0.0


class PerceptionCriteria:
    """Criteria interface for perception evaluation

    Args:
        mode (Optional[Union[str, CriteriaMode]])
        level (Optional[Union[str, CriteriaLevel]])
    """

    def __init__(
        self,
        mode: Optional[Union[str, CriteriaMode]] = None,
        level: Optional[Union[str, CriteriaLevel]] = None,
    ) -> None:
        if mode is None:
            mode: CriteriaMode = CriteriaMode.NUM_FAILED_OBJECT
        elif isinstance(mode, str):
            mode: CriteriaMode = CriteriaMode.from_str(mode)
        assert isinstance(mode, CriteriaMode), f"Invalid type of mode: {type(mode)}"

        if level is None:
            level = CriteriaLevel.EASY
        elif isinstance(level, str):
            level = CriteriaLevel.from_str(level)
        assert isinstance(level, CriteriaLevel), f"Invalid type of level: {type(level)}"

        if mode == CriteriaMode.NUM_FAILED_OBJECT:
            self.method = NumFailObject(level)
        elif mode == CriteriaMode.METRICS_SCORE:
            self.method = MetricsScore(level)

    def get_result(self, frame: PerceptionFrameResult) -> SuccessFail:
        """Returns Success/Fail result from `PerceptionFrameResult`.

        Args:
            frame (PerceptionFrameResult): Frame result of perception evaluation.

        Returns:
            SuccessFail: Success/Fail result.
        """
        return self.method.get_result(frame)
