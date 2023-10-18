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
from dataclasses import field
from functools import singledispatchmethod
import statistics
from typing import ClassVar

from diagnostic_msgs.msg import DiagnosticArray
from example_interfaces.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from rosidl_runtime_py import message_to_ordereddict
from tf_transformations import euler_from_quaternion
from tier4_debug_msgs.msg import Float32Stamped
from tier4_debug_msgs.msg import Int32Stamped

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase


def calc_pose_lateral_distance(relative_pose: PoseStamped) -> float:
    return relative_pose.pose.position.y


def calc_pose_horizontal_distance(relative_pose: PoseStamped) -> float:
    x = relative_pose.pose.position.x
    y = relative_pose.pose.position.y
    return np.sqrt(np.power(x, 2) + np.power(y, 2))


def get_reliability_method(method_name: str | None) -> tuple[str | None, str]:
    if method_name is None:
        return None, "Scenario format error"
    if method_name in ["TP", "NVTL"]:
        return method_name, ""
    return None, f"{method_name} is not valid reliability method"


@dataclass
class Convergence(EvaluationItem):
    name: ClassVar[str] = "Convergence"
    success: bool = True

    def set_frame(
        self,
        lateral_dist: float,
        horizontal_dist: float,
        map_to_baselink: dict,
        exe_time: Float32Stamped,
        iteration_num: Int32Stamped,
    ) -> tuple[dict, Float64]:
        self.total += 1
        frame_success = "Fail"

        msg_lateral_dist = Float64()
        msg_lateral_dist.data = lateral_dist

        exe_time_ms = exe_time.data
        iteration = iteration_num.data

        if (
            abs(lateral_dist) <= self.condition["AllowableDistance"]
            and exe_time_ms <= self.condition["AllowableExeTimeMs"]
            and iteration <= self.condition["AllowableIterationNum"]
        ):
            self.passed += 1
            frame_success = "Success"

        current_rate = self.rate()
        self.success = current_rate >= self.condition["PassRate"]
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {current_rate:.2f}%"

        return {
            "Ego": {"TransformStamped": map_to_baselink},
            "Convergence": {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {
                    "LateralDistance": lateral_dist,
                    "HorizontalDistance": horizontal_dist,
                    "ExeTimeMs": exe_time_ms,
                    "IterationNum": iteration,
                },
            },
        }, msg_lateral_dist


@dataclass
class Reliability(EvaluationItem):
    name: ClassVar[str] = "Reliability"
    success: bool = True
    ng_seq: int = 0
    received_data: list[float] = field(default_factory=list)

    def set_frame(
        self,
        msg: Float32Stamped,
        map_to_baselink: dict,
        reference: Float32Stamped,
    ) -> dict:
        self.total += 1
        frame_success = "Fail"
        self.received_data.append(msg.data)

        # If the likelihood is lower than AllowableLikelihood for NGCount consecutive times, it is assumed to be a failure.
        if self.success:
            # Update nq_seq only while reliability.result is true
            if msg.data >= self.condition["AllowableLikelihood"]:
                self.ng_seq = 0
                frame_success = "Success"
            else:
                self.ng_seq += 1
        self.success = bool(self.ng_seq < self.condition["NGCount"])

        self.summary = f"{self.name} ({self.success_str()}): {self.condition['Method']} Sequential NG Count: {self.ng_seq} (Total Test: {self.total}, Average: {statistics.mean(self.received_data):.5f}, StdDev: {statistics.pstdev(self.received_data):.5f})"
        return {
            "Ego": {"TransformStamped": map_to_baselink},
            "Reliability": {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {
                    "Value": message_to_ordereddict(msg),
                    "Reference": message_to_ordereddict(reference),
                },
            },
        }


@dataclass
class Availability(EvaluationItem):
    name: ClassVar[str] = "NDT Availability"
    success: bool = True
    TARGET_DIAG_NAME: ClassVar[
        str
    ] = "/autoware/localization/node_alive_monitoring/topic_status/topic_state_monitor_ndt_scan_matcher_exe_time: localization_topic_status"
    ERROR_STATUS_LIST: ClassVar[list[str]] = ["Timeout", "NotReceived"]

    def set_frame(self, msg: DiagnosticArray) -> dict:
        include_target_status = False
        # Check if the NDT is available. Note that it does NOT check topic rate itself, but just the availability of the topic
        for diag_status in msg.status:
            if diag_status.name != Availability.TARGET_DIAG_NAME:
                continue
            include_target_status = True
            values = {value.key: value.value for value in diag_status.values}
            # Here we assume that, once a node (e.g. ndt_scan_matcher) fails, it will not be relaunched automatically.
            # On the basis of this assumption, we only consider the latest diagnostics received.
            # Possible status are OK, Timeout, NotReceived, WarnRate, and ErrorRate
            status_str: str | None = values.get("status")
            if status_str is not None:
                if status_str in Availability.ERROR_STATUS_LIST:
                    self.success = False
                    self.summary = f"{self.name} ({self.success_str()}): NDT not available"
                else:
                    self.success = True
                    self.summary = f"{self.name} ({self.success_str()}): NDT available"
            else:
                self.success = False
                self.summary = f"{self.name} ({self.success_str()}): NDT Availability Key Not Found"
            break
        if include_target_status:
            return {
                "Ego": {},
                "Availability": {
                    "Result": {"Total": self.success_str(), "Frame": self.success_str()},
                    "Info": {},
                },
            }
        return {
            "Ego": {},
            "Availability": {
                "Result": {"Total": self.success_str(), "Frame": "Warn"},
                "Info": {"Reason": "diagnostics does not contain localization_topic_status"},
            },
        }


class LocalizationResult(ResultBase):
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__convergence = Convergence(condition=condition["Convergence"])
        self.__reliability = Reliability(condition=condition["Reliability"])
        self.__availability = Availability()

    def update(self) -> None:
        summary_str = f"{self.__convergence.summary}, {self.__reliability.summary}, {self.__availability.summary}"
        if (
            self.__convergence.success
            and self.__reliability.success
            and self.__availability.success
        ):
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    @singledispatchmethod
    def set_frame(self) -> None:
        raise NotImplementedError

    @set_frame.register
    def set_reliability_frame(
        self,
        msg: Float32Stamped,
        map_to_baselink: dict,
        reference: Float32Stamped,
    ) -> None:
        self._frame = self.__reliability.set_frame(msg, map_to_baselink, reference)
        self.update()

    @set_frame.register
    def set_convergence_frame(
        self,
        lateral_dist: float,
        horizontal_dist: float,
        map_to_baselink: dict,
        exe_time: Float32Stamped,
        iteration_num: Int32Stamped,
    ) -> Float64:
        self._frame, msg_lateral_dist = self.__convergence.set_frame(
            lateral_dist,
            horizontal_dist,
            map_to_baselink,
            exe_time,
            iteration_num,
        )
        self.update()
        return msg_lateral_dist

    @set_frame.register
    def set_ndt_availability_frame(self, msg: DiagnosticArray) -> None:
        self._frame = self.__availability.set_frame(msg)
        self.update()
