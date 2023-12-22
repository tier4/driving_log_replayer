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
from copy import deepcopy
from enum import Enum
from numbers import Number
from typing import TYPE_CHECKING

from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.evaluation.matching import MatchingMode
from perception_eval.evaluation.matching.objects_filter import filter_object_results
from perception_eval.evaluation.matching.objects_filter import filter_objects

if TYPE_CHECKING:
    from perception_eval.evaluation import PerceptionFrameResult


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

    def __and__(self, other: SuccessFail) -> SuccessFail:
        return SuccessFail.SUCCESS if self.is_success() and other.is_success() else SuccessFail.FAIL


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
        assert name != "CUSTOM", "If you want to use custom level, use from_number."
        assert name in cls.__members__, "value must be PERFECT, HARD, NORMAL, or EASY"
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
    - METRICS_SCORE_MAPH: mAPH score.
    """

    NUM_TP = "num_tp"
    METRICS_SCORE = "metrics_score"
    METRICS_SCORE_MAPH = "metrics_score_maph"

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
        assert name in cls.__members__, "value must be NUM_TP, METRICS_SCORE, or METRICS_SCORE_MAPH"
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
        # no ground truth and no result is considered as success
        if self.has_objects(frame) is False:
            return SuccessFail.SUCCESS
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


class MetricsScoreMAPH(CriteriaMethodImpl):
    name = CriteriaMethod.METRICS_SCORE_MAPH

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        assert frame.metrics_score.evaluation_task.is_3d(), "Evaluation task must be 3D for MAPH."
        scores = [
            map_.maph
            for map_ in frame.metrics_score.maps
            if map_.maph != float("inf") and map_.matching_mode == MatchingMode.CENTERDISTANCE
        ]

        return 100.0 * sum(scores) / len(scores) if len(scores) != 0 else 0.0


class CriteriaFilter:
    def __init__(self, distance_range: tuple(Number, Number) = None) -> None:
        self.distance_range = distance_range

    def is_all_none(self) -> bool:
        """
        Return True if all filter params are None.

        Returns
        -------
            bool: True if all filter params are None.
        """
        return self.distance_range is None

    def filter_frame_results(self, frame: PerceptionFrameResult) -> PerceptionFrameResult:
        """
        Filter PerceptionFrameResult by distance range.

        If all filter params are None, do nothing and return original frame result.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            PerceptionFrameResult: Filtered result.
        """
        if self.is_all_none:
            return frame

        if self.distance_range is not None:
            min_distance_list = [self.Distance[0]] * len(frame.target_labels)
            max_distance_list = [self.Distance[1]] * len(frame.target_labels)
        else:
            min_distance_list = None
            max_distance_list = None

        object_results = filter_object_results(
            frame.object_results,
            frame.target_labels,
            max_distance_list=max_distance_list,
            min_distance_list=min_distance_list,
            ego2map=frame.frame_ground_truth.ego2map,
        )
        frame_ground_truth = frame.frame_ground_truth

        frame_ground_truth.objects = filter_objects(
            frame_ground_truth.objects,
            is_gt=True,
            target_labels=frame.target_labels,
            max_distance_list=max_distance_list,
            min_distance_list=min_distance_list,
            ego2map=frame_ground_truth.ego2map,
        )

        filtered_frame = deepcopy(frame)
        filtered_frame.object_results = object_results
        filtered_frame.frame_ground_truth = frame_ground_truth
        filtered_frame.evaluate_frame(frame_ground_truth)

        return filtered_frame


class PerceptionCriteria:
    """
    Criteria interface for perception evaluation.

    Args:
    ----
        methods (str | list[str] | CriteriaMethod | list[CriteriaMethod] | None): List of criteria method instances or names.
            If None, `CriteriaMethod.NUM_TP` is used. Defaults to None.
        levels (str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel]): Criteria level instance or name.
            If None, `CriteriaLevel.Easy` is used. Defaults to None.
        distance_range (tuple[Number, Number] | None): Range of distance to filter frame result. Defaults to None.
    """

    def __init__(
        self,
        methods: str | list[str] | CriteriaMethod | list[CriteriaMethod] | None = None,
        levels: str
        | list[str]
        | Number
        | list[Number]
        | CriteriaLevel
        | list[CriteriaLevel]
        | None = None,
        distance_range: tuple[Number, Number] | None = None,
    ) -> None:
        methods = [CriteriaMethod.NUM_TP] if methods is None else self.load_methods(methods)
        levels = [CriteriaLevel.EASY] if levels is None else self.load_levels(levels)

        assert len(methods) == len(
            levels,
        ), f"Number of CriteriaMethod and CriteriaLevel must be same. Current methods: {methods}, levels: {levels}"

        self.methods = []
        for method, level in zip(methods, levels):
            if method == CriteriaMethod.NUM_TP:
                self.methods.append(NumTP(level))
            elif method == CriteriaMethod.METRICS_SCORE:
                self.methods.append(MetricsScore(level))
            elif method == CriteriaMethod.METRICS_SCORE_MAPH:
                self.methods.append(MetricsScoreMAPH(level))
            else:
                error_msg: str = f"Unsupported method: {method}"
                raise NotImplementedError(error_msg)

        self.criteria_filter = CriteriaFilter(distance_range)

    @staticmethod
    def load_methods(
        methods_input: str | list[str] | CriteriaMethod | list[CriteriaMethod],
    ) -> list[CriteriaMethod]:
        """
        Load `CriteriaMethod` enum.

        Args:
        ----
            methods_input (str | list[str] | CriteriaMethod | list[CriteriaMethod]): Criteria method instance or name.

        Returns:
        -------
            list[CriteriaMethod]: Instance.
        """
        if isinstance(methods_input, str):
            loaded_methods = [CriteriaMethod.from_str(methods_input)]
        elif isinstance(methods_input, CriteriaMethod):
            loaded_methods = [methods_input]
        elif isinstance(methods_input, list):
            if isinstance(methods_input[0], str):
                loaded_methods = [CriteriaMethod.from_str(method) for method in methods_input]
            elif isinstance(methods_input[0], CriteriaMethod):
                loaded_methods = methods_input

        for method in loaded_methods:
            assert isinstance(method, CriteriaMethod), f"Invalid type of method: {type(method)}"
        return loaded_methods

    @staticmethod
    def load_levels(
        levels_input: str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel],
    ) -> list[CriteriaLevel]:
        """
        Load `CriteriaLevel`.

        Args:
        ----
            levels_input (str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel]): Criteria level instance, name or value.

        Returns:
        -------
            list[CriteriaLevel]: Instance.
        """
        if isinstance(levels_input, str):
            levels_output = [CriteriaLevel.from_str(levels_input)]
        elif isinstance(levels_input, Number):
            levels_output = [CriteriaLevel.from_number(levels_input)]
        elif isinstance(levels_input, CriteriaLevel):
            levels_output = [levels_input]
        elif isinstance(levels_input, list):
            if isinstance(levels_input[0], str):
                levels_output = [CriteriaLevel.from_str(level) for level in levels_input]
            elif isinstance(levels_input[0], Number):
                levels_output = [CriteriaLevel.from_number(level) for level in levels_input]
            elif isinstance(levels_input[0], CriteriaLevel):
                levels_output = levels_input
        for level in levels_output:
            assert isinstance(level, CriteriaLevel), f"Invalid type of level: {type(level)}"
        return levels_output

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
        frame = self.criteria_filter.filter_frame_results(frame)

        result: SuccessFail = SuccessFail.SUCCESS

        for method in self.methods:
            result &= method.get_result(frame)
        return result
