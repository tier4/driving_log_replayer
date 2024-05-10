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
import math
from numbers import Number
from typing import TYPE_CHECKING

import numpy as np
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

    def is_valid(self, score: Number, is_greater: bool) -> bool:
        """
        Return whether the score satisfied the level.

        Args:
        ----
            score (Number): Calculated score.
            is_greater (bool, optional): Whether the score is valid when it is greater than `self.value`. Defaults to True.

        Returns:
        -------
            bool: Whether the score satisfied the level.

        """
        return score >= self.value if is_greater else score <= self.value

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
    - LABEL: Whether label is correct or not.
    - METRICS_SCORE: Accuracy score for classification, otherwise mAP score is used.
    - METRICS_SCORE_MAPH: mAPH score.
    """

    NUM_TP = "num_tp"
    LABEL = "label"
    VELOCITY_X = "velocity_x"
    VELOCITY_Y = "velocity_y"
    VELOCITY_NORM = "velocity_norm"
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

    def get_result(self, frame: PerceptionFrameResult) -> SuccessFail | None:
        """
        Return `SuccessFail` instance from the frame result.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            SuccessFail: Success or fail.

        """
        # No ground truth and No result is considered as Not Available, return None.
        if self.has_objects(frame) is False:
            return None
        score: float = self.calculate_score(frame)
        return (
            SuccessFail.SUCCESS if self.level.is_valid(score, self.is_greater) else SuccessFail.FAIL
        )

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

    @property
    @abstractmethod
    def is_greater(self) -> bool:
        """
        Whether the score is valid when it is greater than a threshold.

        Returns
        -------
            bool: True means it is valid if score >= threshold .

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

    @property
    def is_greater(self) -> bool:
        return True


class Label(CriteriaMethodImpl):
    name = CriteriaMethod.LABEL

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        is_label_corrects = [
            result.is_label_correct
            for result in frame.object_results
            if result.ground_truth_object is not None
        ]

        return 100.0 if len(is_label_corrects) == 0 else 100.0 * np.mean(is_label_corrects)

    @property
    def is_greater(self) -> bool:
        return True


class VelocityX(CriteriaMethodImpl):
    name = CriteriaMethod.VELOCITY_X

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        errors = []
        for result in frame.object_results:
            if result.ground_truth_object is not None:
                est_vx = result.estimated_object.state.velocity[0]
                gt_vx = result.ground_truth_object.state.velocity[0]
                err = abs(gt_vx - est_vx)
                errors.append(err)
        return 0.0 if len(errors) == 0 else np.mean(errors)

    @property
    def is_greater(self) -> bool:
        return False


class VelocityY(CriteriaMethodImpl):
    name = CriteriaMethod.VELOCITY_Y

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        errors = []
        for result in frame.object_results:
            if result.ground_truth_object is not None:
                est_vy = result.estimated_object.state.velocity[1]
                gt_vy = result.ground_truth_object.state.velocity[1]
                err = abs(gt_vy - est_vy)
                errors.append(err)
        return 0.0 if len(errors) == 0 else np.mean(errors)

    @property
    def is_greater(self) -> bool:
        return False


class VelocityNorm(CriteriaMethodImpl):
    name = CriteriaMethod.VELOCITY_NORM

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        errors = []
        for result in frame.object_results:
            if result.ground_truth_object is not None:
                est_norm = math.hypot(result.estimated_object.state.velocity[:2])
                gt_norm = math.hypot(result.ground_truth_object.state.velocity[:2])
                err = abs(gt_norm - est_norm)
                errors.append(err)
        return 0.0 if len(errors) == 0 else np.mean(errors)

    @property
    def is_greater(self) -> bool:
        return False


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

    @property
    def is_greater(self) -> bool:
        return True


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

    @property
    def is_greater(self) -> bool:
        return True


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

    def filter_frame_result(self, frame: PerceptionFrameResult) -> PerceptionFrameResult:
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
        if self.is_all_none():
            return frame

        filtered_frame = deepcopy(frame)

        if self.distance_range is not None:
            min_distance_list = [self.distance_range[0]] * len(filtered_frame.target_labels)
            max_distance_list = [self.distance_range[1]] * len(filtered_frame.target_labels)
        else:
            min_distance_list = None
            max_distance_list = None

        object_results = filter_object_results(
            filtered_frame.object_results,
            filtered_frame.target_labels,
            max_distance_list=max_distance_list,
            min_distance_list=min_distance_list,
            transforms=filtered_frame.frame_ground_truth.transforms,
        )
        frame_ground_truth = filtered_frame.frame_ground_truth

        frame_ground_truth.objects = filter_objects(
            frame_ground_truth.objects,
            is_gt=True,
            target_labels=frame.target_labels,
            max_distance_list=max_distance_list,
            min_distance_list=min_distance_list,
            transforms=filtered_frame.frame_ground_truth.transforms,
        )

        filtered_frame.object_results = object_results
        filtered_frame.frame_ground_truth = frame_ground_truth
        filtered_frame.evaluate_frame(frame_ground_truth.objects)

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
        levels: (
            str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel] | None
        ) = None,
        distance_range: tuple[Number, Number] | None = None,
    ) -> None:
        methods = [CriteriaMethod.NUM_TP] if methods is None else self.load_methods(methods)
        levels = [CriteriaLevel.EASY] if levels is None else self.load_levels(levels)

        assert len(methods) == len(
            levels,
        ), f"Number of CriteriaMethod and CriteriaLevel must be same. Current methods: {methods}, levels: {levels}"

        self.methods = []
        for method, level in zip(methods, levels, strict=True):
            if method == CriteriaMethod.NUM_TP:
                self.methods.append(NumTP(level))
            elif method == CriteriaMethod.LABEL:
                self.methods.append(Label(level))
            elif method == CriteriaMethod.VELOCITY_X:
                self.methods.append(VelocityX(level))
            elif method == CriteriaMethod.VELOCITY_Y:
                self.methods.append(VelocityY(level))
            elif method == CriteriaMethod.VELOCITY_NORM:
                self.methods.append(VelocityNorm(level))
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

    def get_result(
        self,
        frame: PerceptionFrameResult,
    ) -> tuple[SuccessFail | None, PerceptionFrameResult]:
        """
        Return Success/Fail result from `PerceptionFrameResult`.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result of perception evaluation.

        Returns:
        -------
            tuple[SuccessFail, PerceptionFrameResult]: Success/Fail result and frame result.

        """
        ret_frame = self.criteria_filter.filter_frame_result(frame)

        result: SuccessFail = SuccessFail.SUCCESS

        for method in self.methods:
            method_result = method.get_result(ret_frame)
            if method_result is None:
                return None, ret_frame
            result &= method.get_result(ret_frame)
        return result, ret_frame
