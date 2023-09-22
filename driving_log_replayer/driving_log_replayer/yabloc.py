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

from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import TopicResult


@dataclass
class AvailabilityResult(TopicResult):
    name: ClassVar[str] = "NDT Availability"
    TARGET_DIAG_NAME: ClassVar[str] = "yabloc_monitor: yabloc_status"

    def set_frame(self, msg: DiagnosticArray) -> dict:
        include_target_status = False
        # Check if the NDT is available. Note that it does NOT check topic rate itself, but just the availability of the topic
        for diag_status in msg.status:
            if diag_status.name != AvailabilityResult.TARGET_DIAG_NAME:
                continue
            include_target_status = True
            values = {value.key: value.value for value in diag_status.values}
            # Here we assume that, once a node (e.g. ndt_scan_matcher) fails, it will not be relaunched automatically.
            # On the basis of this assumption, we only consider the latest diagnostics received.
            # Possible status are OK, Timeout, NotReceived, WarnRate, and ErrorRate
            if values["status"] in AvailabilityResult.ERROR_STATUS_LIST:
                self.success = False
                self.summary = f"{self.name} ({self.success_str()}): NDT not available"
            else:
                self.success = True
                self.summary = f"{self.name} ({self.success_str()}): NDT available"
        if include_target_status:
            return {
                "Availability": {
                    "Result": self.success_str(),
                    "Info": [
                        {},
                    ],
                },
            }
        return {
            "Availability": {
                "Result": "Fail",
                "Info": [
                    {"Reason": "diagnostics does not contain localization_topic_status"},
                ],
            },
        }


class YabLocResult(ResultBase):
    def __init__(self) -> None:
        super().__init__()
        self.__availability = AvailabilityResult()
        # availability
        self.__yabloc_availability_result = False
        self.__yabloc_availability_msg = "NotTested"

    def update(self) -> None:
        if self.__yabloc_availability_result:
            yabloc_availability_summary = (
                f"YabLoc Availability (Passed): {self.__yabloc_availability_msg}"
            )
        else:
            yabloc_availability_summary = (
                f"YabLoc Availability (Failed): {self.__yabloc_availability_msg}"
            )
        summary_str = f"{yabloc_availability_summary}"
        if self.__yabloc_availability_result:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(self, msg: DiagnosticArray) -> None:
        for diag_status in msg.status:
            out_frame = {"Ego": {}}
            if diag_status.name != "yabloc_monitor: yabloc_status":
                continue
            values = {value.key: value.value for value in diag_status.values}
            self.__yabloc_availability_result = values["Availability"] == "OK"
            self.__yabloc_availability_msg = values["Availability"]
            out_frame["Availability"] = {
                "Result": "Success" if self.__yabloc_availability_result else "Fail",
                "Info": [],
            }
            self._frame = out_frame
            self.update()
