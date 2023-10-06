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

from __future__ import annotations

from abc import ABC
from abc import abstractmethod
from enum import Enum
from numbers import Number
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from perception_eval.evaluation import PerceptionFrameResult

from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.evaluation.matching import MatchingMode


class SuccessFail(Enum):
    """Enum object represents evaluated result is success or fail."""

    SUCCESS = "Success"
    FAIL = "Fail"

    def __str__(self) -> str:
        return self.value

    def is_success(self) -> bool:
        """
        Return whether success or fail.

        Returns
        -------
            bool: Success or fail.
        """
        return self == SuccessFail.SUCCESS


class CriteriaLevel(Enum):
    """
    Enum object represents criteria level.

    PERFECT == 100.0
    HARD    >= 75.0
    NORMAL  >= 50.0
    EASY    >= 25.0

    CUSTOM  >= SCORE YOU SPECIFIED [0.0, 100.0]
    """

    PERFECT = 100.0
    HARD = 75.0
    NORMAL = 50.0
    EASY = 25.0

    CUSTOM = None

    def is_valid(self, score: Number) -> bool:
        """
        Return whether the score satisfied the level.

        Args:
        ----
            score (Number): Calculated score.

        Returns:
        -------
            bool: Whether the score satisfied the level.
        """
        return score >= self.value

    @classmethod
    def from_str(cls, value: str) -> CriteriaLevel:
        """
        Construct instance from.

        Args:
        ----
            value (str): _description_

        Returns:
        -------
            CriteriaLevel: _description_
        """
        name: str = value.upper()
        assert name != "CUSTOM", "If you want to use custom level, input value [0.0, 100.0]."
        return cls.__members__[name]

    @classmethod
    def from_number(cls, value: Number) -> CriteriaLevel:
        """
        Construct `CriteriaLevel.CUSTOM` with custom value.

        Args:
        ----
            value (Number): Level value which is must be [0.0, 100.0].

        Returns:
        -------
            CriteriaLevel: `CriteriaLevel.CUSTOM` with custom value.
        """
        min_range = 0.0
        max_range = 100.0
        assert (
            min_range <= value <= max_range
        ), f"Custom level must be [0.0, 100.0], but got {value}."
        cls.CUSTOM._value_ = float(value)
        return cls.CUSTOM


class CriteriaMethod(Enum):
    """
    Enum object represents methods of criteria .

    - NUM_TP: Number of TP (or TN).
    - METRICS_SCORE: Accuracy score for classification, otherwise mAP score is used.
    """

    NUM_TP = "num_tp"
    METRICS_SCORE = "metrics_score"

    @classmethod
    def from_str(cls, value: str) -> CriteriaMethod:
        """
        Construct instance from name in string.

        Args:
        ----
            value (str): Name of enum.

        Returns:
        -------
            CriteriaMode: `CriteriaMode` instance.
        """
        name: str = value.upper()
        return cls.__members__[name]


class CriteriaMethodImpl(ABC):
    """
    Class to define implementation for each criteria.

    Args:
    ----
        level (CriteriaLevel): Level of criteria.
    """

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__()
        self.level: CriteriaLevel = level

    def get_result(self, frame: PerceptionFrameResult) -> SuccessFail:
        """
        Return `SuccessFail` instance from the frame result.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            SuccessFail: Success or fail.
        """
        if self.has_objects(frame) is False:
            return SuccessFail.FAIL
        score: float = self.calculate_score(frame)
        return SuccessFail.SUCCESS if self.level.is_valid(score) else SuccessFail.FAIL

    @staticmethod
    def has_objects(frame: PerceptionFrameResult) -> bool:
        """
        Return whether the frame result contains at least one objects.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            bool: Whether the frame result has objects is.
        """
        num_success: int = frame.pass_fail_result.get_num_success()
        num_fail: int = frame.pass_fail_result.get_num_fail()
        return num_success + num_fail > 0

    @staticmethod
    @abstractmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        """
        Calculate score depending on the method.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            float: Calculated score.
        """


class NumTP(CriteriaMethodImpl):
    name = CriteriaMethod.NUM_TP

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        num_success: int = frame.pass_fail_result.get_num_success()
        num_objects: int = num_success + frame.pass_fail_result.get_num_fail()
        return 100.0 * num_success / num_objects if num_objects != 0 else 0.0


class MetricsScore(CriteriaMethodImpl):
    name = CriteriaMethod.METRICS_SCORE

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
    """
    Criteria interface for perception evaluation.

    Args:
    ----
        method (str | CriteriaMethod | None): Criteria method instance or name.
            If None, `CriteriaMethod.NUM_TP` is used. Defaults to None.
        level (str | Number | CriteriaLevel | None): Criteria level instance or name.
            If None, `CriteriaLevel.Easy` is used. Defaults to None.
    """

    def __init__(
        self,
        method: str | CriteriaMethod | None = None,
        level: str | Number | CriteriaLevel | None = None,
    ) -> None:
        method = CriteriaMethod.NUM_TP if method is None else self.load_method(method)
        level = CriteriaLevel.EASY if level is None else self.load_level(level)

        if method == CriteriaMethod.NUM_TP:
            self.method = NumTP(level)
        elif method == CriteriaMethod.METRICS_SCORE:
            self.method = MetricsScore(level)

    @staticmethod
    def load_method(method: str | CriteriaMethod) -> CriteriaMethod:
        """
        Load `CriteriaMethod` enum.

        Args:
        ----
            method (str | CriteriaMethod): Criteria method instance or name.

        Returns:
        -------
            CriteriaMethod: Instance.
        """
        if isinstance(method, str):
            method: CriteriaMethod = CriteriaMethod.from_str(method)
        assert isinstance(method, CriteriaMethod), f"Invalid type of method: {type(method)}"
        return method

    @staticmethod
    def load_level(level: str | Number | CriteriaLevel) -> CriteriaLevel:
        """
        Load `CriteriaLevel`.

        Args:
        ----
            level (str | Number | CriteriaLevel): Criteria level instance, name or value.

        Returns:
        -------
            CriteriaLevel: Instance.
        """
        if isinstance(level, str):
            level = CriteriaLevel.from_str(level)
        elif isinstance(level, Number):
            level = CriteriaLevel.from_number(level)
        assert isinstance(level, CriteriaLevel), f"Invalid type of level: {type(level)}"
        return level

    def get_result(self, frame: PerceptionFrameResult) -> SuccessFail:
        """
        Return Success/Fail result from `PerceptionFrameResult`.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result of perception evaluation.

        Returns:
        -------
            SuccessFail: Success/Fail result.
        """
        return self.method.get_result(frame)
