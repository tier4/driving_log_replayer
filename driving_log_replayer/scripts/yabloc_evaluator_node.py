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

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.result import ResultBase


class YabLocResult(ResultBase):
    def __init__(self):
        super().__init__()
        # availability
        self.__yabloc_availability_result = False
        self.__yabloc_availability_msg = "NotTested"

    def update(self):
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

    def set_frame(self, msg: DiagnosticArray):
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


class YabLocEvaluator(DLREvaluator):
    def __init__(self, name: str):
        super().__init__(name)
        self.check_scenario()

        self.__result = YabLocResult()

        self.__sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb,
            1,
        )

    def check_scenario(self) -> None:
        pass

    def diagnostics_cb(self, msg: DiagnosticArray):
        self.__result.set_frame(msg)
        self._result_writer.write(self.__result)


@evaluator_main
def main():
    return YabLocEvaluator("yabloc_evaluator")


if __name__ == "__main__":
    main()
