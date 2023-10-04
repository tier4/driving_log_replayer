from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from numbers import Number
from typing import Optional, Union

from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.evaluation import PerceptionFrameResult
from perception_eval.evaluation.matching import MatchingMode


class SuccessFail(Enum):
    """Enum object represents evaluated result is success or fail."""

    SUCCESS = "Success"
    FAIL = "Fail"

    def __str__(self) -> str:
        return self.value

    def is_success(self) -> bool:
        """Returns whether success or fail.

        Returns:
            bool: Success or fail.
        """
        return self == SuccessFail.SUCCESS


class CriteriaLevel(Enum):
    """Enum object represents criteria level."""

    PERFECT = 100.0
    HARD = 90.0
    NORMAL = 75.0
    EASY = 50.0

    CUSTOM = None

    def is_valid(self, score: Number) -> bool:
        """Returns whether the score satisfied the level.

        Args:
            score (Number): Calculated score.

        Returns:
            bool: Whether the score satisfied the level.
        """
        return score >= self.value

    @classmethod
    def from_str(cls, value: str) -> CriteriaLevel:
        """Constructs instance from

        Args:
            value (str): _description_

        Returns:
            CriteriaLevel: _description_
        """
        name: str = value.upper()
        assert name != "CUSTOM", "If you want to use custom level, input value [0.0, 100.0]."
        return cls.__members__[name]

    @classmethod
    def from_number(cls, value: Number) -> CriteriaLevel:
        """Constructs `CriteriaLevel.CUSTOM` with custom value.

        Args:
            value (Number): Level value which is must be [0.0, 100.0].

        Returns:
            CriteriaLevel: `CriteriaLevel.CUSTOM` with custom value.
        """
        assert 0.0 <= value <= 100.0, f"Custom level must be [0.0, 100.0], but got {value}."
        cls.CUSTOM._value_ = float(value)
        return cls.CUSTOM


class CriteriaMode(Enum):
    """Enum object represents criteria mode."""

    NUM_FAILED_OBJECT = "num_failed_object"
    METRICS_SCORE = "metrics_score"

    @classmethod
    def from_str(cls, value: str) -> CriteriaMode:
        """Constructs instance from name in string.

        Args:
            value (str): Name of enum.

        Returns:
            CriteriaMode: `CriteriaMode` instance.
        """
        name: str = value.upper()
        return cls.__members__[name]


class CriteriaMethod(ABC):
    """Class to define implementation for each criteria."""

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__()
        self.level: CriteriaLevel = level

    def get_result(self, frame: PerceptionFrameResult) -> SuccessFail:
        """Returns `SuccessFail` instance from the frame result.

        Args:
            frame (PerceptionFrameResult): Frame result.

        Returns:
            SuccessFail: Success or fail.
        """
        if self.has_objects(frame) is False:
            return SuccessFail.FAIL
        score: float = self.calculate_score(frame)
        return SuccessFail.SUCCESS if self.level.is_valid(score) else SuccessFail.FAIL

    @staticmethod
    def has_objects(frame: PerceptionFrameResult) -> bool:
        """Returns whether the frame result contains at least one objects.

        Args:
            frame (PerceptionFrameResult): Frame result.

        Returns:
            bool: Whether the frame result has objects is.
        """
        num_success: int = frame.pass_fail_result.get_num_success()
        num_fail: int = frame.pass_fail_result.get_num_fail()
        return num_success + num_fail > 0

    @staticmethod
    @abstractmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        """Calculates score depending on the method.

        Args:
            frame (PerceptionFrameResult): Frame result.

        Returns:
            float: Calculated score.
        """
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
            scores = [
                map_.map
                for map_ in frame.metrics_score.maps
                if map_.map != float("inf") and map_.matching_mode == MatchingMode.CENTERDISTANCE
            ]

        return 100.0 * sum(scores) / len(scores) if len(scores) != 0 else 0.0


class PerceptionCriteria:
    """Criteria interface for perception evaluation.

    Args:
        mode (Optional[Union[str, CriteriaMode]])
        level (Optional[Union[str, Number, CriteriaLevel]])
    """

    def __init__(
        self,
        mode: Optional[Union[str, CriteriaMode]] = None,
        level: Optional[Union[str, Number, CriteriaLevel]] = None,
    ) -> None:
        mode = CriteriaMode.NUM_FAILED_OBJECT if mode is None else self.load_mode(mode)
        level = CriteriaLevel.EASY if level is None else self.load_level(level)

        if mode == CriteriaMode.NUM_FAILED_OBJECT:
            self.method = NumFailObject(level)
        elif mode == CriteriaMode.METRICS_SCORE:
            self.method = MetricsScore(level)

    @staticmethod
    def load_mode(mode: Union[str, CriteriaMode]) -> CriteriaMode:
        """Load `CriteriaMode`.

        Args:
            mode (Optional[Union[str, CriteriaMode]]): Criteria mode instance or name.

        Returns:
            CriteriaMode: Instance.
        """
        if isinstance(mode, str):
            mode: CriteriaMode = CriteriaMode.from_str(mode)
        assert isinstance(mode, CriteriaMode), f"Invalid type of mode: {type(mode)}"
        return mode

    @staticmethod
    def load_level(level: Union[str, Number, CriteriaLevel]) -> CriteriaLevel:
        """Load `CriteriaLevel`.

        Args:
            level (Optional[Union[str, Number, CriteriaLevel]]): Criteria level instance, name or value.

        Returns:
            CriteriaLevel: Instance.
        """
        if isinstance(level, str):
            level = CriteriaLevel.from_str(level)
        elif isinstance(level, Number):
            level = CriteriaLevel.from_number(level)
        assert isinstance(level, CriteriaLevel), f"Invalid type of level: {type(level)}"
        return level

    def get_result(self, frame: PerceptionFrameResult) -> SuccessFail:
        """Returns Success/Fail result from `PerceptionFrameResult`.

        Args:
            frame (PerceptionFrameResult): Frame result of perception evaluation.

        Returns:
            SuccessFail: Success/Fail result.
        """
        return self.method.get_result(frame)
