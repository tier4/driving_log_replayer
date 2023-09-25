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


class ArTagBasedLocalizerResult(ResultBase):
    def __init__(self) -> None:
        super().__init__()
        # availability
        self.__ar_tag_based_localizer_availability_result = False
        self.__ar_tag_based_localizer_availability_msg = "NotTested"

    def update(self) -> None:
        if self.__ar_tag_based_localizer_availability_result:
            ar_tag_based_localizer_availability_summary = (
                f"ArTagBasedLocalizer Availability (Passed): {self.__ar_tag_based_localizer_availability_msg}"
            )
        else:
            ar_tag_based_localizer_availability_summary = (
                f"ArTagBasedLocalizer Availability (Failed): {self.__ar_tag_based_localizer_availability_msg}"
            )
        summary_str = f"{ar_tag_based_localizer_availability_summary}"
        if self.__ar_tag_based_localizer_availability_result:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(self, msg: DiagnosticArray) -> None:
        for diag_status in msg.status:
            out_frame = {"Ego": {}}
            if diag_status.name != "ar_tag_based_localizer_monitor: ar_tag_based_localizer_status":
                continue
            values = {value.key: value.value for value in diag_status.values}
            self.__ar_tag_based_localizer_availability_result = values["Availability"] == "OK"
            self.__ar_tag_based_localizer_availability_msg = values["Availability"]
            out_frame["Availability"] = {
                "Result": "Success" if self.__ar_tag_based_localizer_availability_result else "Fail",
                "Info": [],
            }
            self._frame = out_frame
            self.update()


class ArTagBasedLocalizerEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.check_scenario()

        self.__result = ArTagBasedLocalizerResult()

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
    return ArTagBasedLocalizerEvaluator("ar_tag_based_localizer_evaluator")


if __name__ == "__main__":
    main()
