#!/usr/bin/env python3

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

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.result import ResultBase


class EagleyeResult(ResultBase):
    def __init__(self) -> None:
        super().__init__()
        # availability
        self.__eagleye_availability_result = False
        self.__eagleye_availability_msg = "NotTested"

    def update(self) -> None:
        if self.__eagleye_availability_result:
            eagleye_availability_summary = (
                f"Eagleye Availability (Passed): {self.__eagleye_availability_msg}"
            )
        else:
            eagleye_availability_summary = (
                f"Eagleye Availability (Failed): {self.__eagleye_availability_msg}"
            )
        summary_str = f"{eagleye_availability_summary}"
        if self.__eagleye_availability_result:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(self, msg: DiagnosticArray) -> None:
        for diag_status in msg.status:
            out_frame = {"Ego": {}}
            if diag_status.name != "monitor: eagleye_enu_absolute_pos_interpolate":
                continue
            self.__eagleye_availability_result = diag_status.level == DiagnosticStatus.OK
            self.__eagleye_availability_msg = diag_status.message
            out_frame["Availability"] = {
                "Result": "Success" if self.__eagleye_availability_result else "Fail",
                "Info": [],
            }
            self._frame = out_frame
            self.update()


class EagleyeEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.check_scenario()

        self.__result = EagleyeResult()

        self.__sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb,
            1,
        )

    def check_scenario(self) -> None:
        pass

    def diagnostics_cb(self, msg: DiagnosticArray) -> None:
        self.__result.set_frame(msg)
        self._result_writer.write_result(self.__result)


@evaluator_main
def main() -> DLREvaluator:
    return EagleyeEvaluator("eagleye_evaluator")


if __name__ == "__main__":
    main()
