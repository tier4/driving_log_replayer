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
import statistics
from typing import ClassVar
from typing import Literal

from diagnostic_msgs.msg import DiagnosticStatus
from example_interfaces.msg import Float64
from pydantic import BaseModel
from rosidl_runtime_py import message_to_ordereddict
from tier4_debug_msgs.msg import Float32Stamped
from tier4_debug_msgs.msg import Int32Stamped

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase
from driving_log_replayer.scenario import InitialPose
from driving_log_replayer.scenario import number
from driving_log_replayer.scenario import Scenario


class ConvergenceCondition(BaseModel):
    AllowableExeTimeMs: number
    AllowableIterationNum: int
    PassRate: number


class ReliabilityCondition(BaseModel):
    Method: Literal["NVTL", "TP"]
    AllowableLikelihood: number
    NGCount: int


class Conditions(BaseModel):
    Convergence: ConvergenceCondition
    Reliability: ReliabilityCondition


class Evaluation(BaseModel):
    UseCaseName: Literal["localization"]
    UseCaseFormatVersion: Literal["1.2.0", "1.3.0"]
    Conditions: Conditions
    InitialPose: InitialPose | None


class LocalizationScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Convergence(EvaluationItem):
    name: str = "Convergence"

    def set_frame(
        self,
        map_to_baselink: dict,
        exe_time: Float32Stamped,
        iteration_num: Int32Stamped,
    ) -> dict:
        self.condition: ConvergenceCondition
        self.total += 1
        frame_success = "Fail"

        exe_time_ms = exe_time.data
        iteration = iteration_num.data

        if (
            exe_time_ms <= self.condition.AllowableExeTimeMs
            and iteration <= self.condition.AllowableIterationNum
        ):
            self.passed += 1
            frame_success = "Success"

        current_rate = self.rate()
        self.success = current_rate >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {current_rate:.2f}%"

        return {
            "Ego": {"TransformStamped": map_to_baselink},
            "Convergence": {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {
                    "ExeTimeMs": exe_time_ms,
                    "IterationNum": iteration,
                },
            },
        }


@dataclass
class Reliability(EvaluationItem):
    name: str = "Reliability"
    ng_seq: int = 0
    received_data: list[float] = field(default_factory=list)

    def set_frame(
        self,
        msg: Float32Stamped,
        map_to_baselink: dict,
        reference: Float32Stamped,
    ) -> dict:
        self.condition: ReliabilityCondition
        self.total += 1
        frame_success = "Fail"
        self.received_data.append(msg.data)

        # If the likelihood is lower than AllowableLikelihood for NGCount consecutive times, it is assumed to be a failure.
        if self.ng_seq < self.condition.NGCount:
            # Update nq_seq only while reliability.result is true
            if msg.data >= self.condition.AllowableLikelihood:
                self.ng_seq = 0
                frame_success = "Success"
            else:
                self.ng_seq += 1
        self.success = bool(self.ng_seq < self.condition.NGCount)

        self.summary = f"{self.name} ({self.success_str()}): {self.condition.Method} Sequential NG Count: {self.ng_seq} (Total Test: {self.total}, Average: {statistics.mean(self.received_data):.5f}, StdDev: {statistics.pstdev(self.received_data):.5f})"
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
    name: str = "NDT Availability"
    ERROR_STATUS_LIST: ClassVar[list[str]] = ["Timeout", "NotReceived"]

    def set_frame(self, diag_status: DiagnosticStatus) -> dict:
        # Check if the NDT is available. Note that it does NOT check topic rate itself, but just the availability of the topic
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
        return {
            "Ego": {},
            "Availability": {
                "Result": {
                    "Total": self.success_str(),
                    "Frame": self.success_str(),
                },
                "Info": {},
            },
        }


class LocalizationResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__convergence = Convergence(condition=condition.Convergence)
        self.__reliability = Reliability(condition=condition.Reliability)
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

    def set_frame(self) -> None:
        pass

    def set_reliability_frame(
        self,
        msg: Float32Stamped,
        map_to_baselink: dict,
        reference: Float32Stamped,
    ) -> None:
        self._frame = self.__reliability.set_frame(msg, map_to_baselink, reference)
        self.update()

    def set_convergence_frame(
        self,
        map_to_baselink: dict,
        exe_time: Float32Stamped,
        iteration_num: Int32Stamped,
    ) -> None:
        self._frame = self.__convergence.set_frame(
            map_to_baselink,
            exe_time,
            iteration_num,
        )
        self.update()

    def set_ndt_availability_frame(self, diag_status: DiagnosticStatus) -> None:
        self._frame = self.__availability.set_frame(diag_status)
        self.update()
