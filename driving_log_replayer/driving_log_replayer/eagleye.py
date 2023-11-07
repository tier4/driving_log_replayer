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
from typing import ClassVar

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus

from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase


@dataclass
class Availability(EvaluationItem):
    name: str = "Eagleye Availability"
    TARGET_DIAG_NAME: ClassVar[str] = "monitor: eagleye_enu_absolute_pos_interpolate"

    def set_frame(self, msg: DiagnosticArray) -> dict:
        include_target_status = False
        diag_status: DiagnosticStatus
        for diag_status in msg.status:
            if diag_status.name != Availability.TARGET_DIAG_NAME:
                continue
            include_target_status = True
            self.success = diag_status.level == DiagnosticStatus.OK
            self.summary = f"{self.name} ({self.success_str()}): {diag_status.message}"
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
                "Info": {
                    "Reason": "diagnostics does not contain eagleye_enu_absolute_pos_interpolate",
                },
            },
        }


class EagleyeResult(ResultBase):
    def __init__(self) -> None:
        super().__init__()
        self.__availability = Availability()

    def update(self) -> None:
        summary_str = f"{self.__availability.summary}"
        if self.__availability.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(self, msg: DiagnosticArray) -> None:
        self._frame = self.__availability.set_frame(msg)
        self.update()
