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

from log_evaluator.evaluator import DLREvaluator
from log_evaluator.evaluator import evaluator_main
from log_evaluator.yabloc import YabLocResult
from log_evaluator.yabloc import YablocScenario

TARGET_DIAG_NAME = "yabloc_monitor: yabloc_status"


class YabLocEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, YablocScenario, YabLocResult)
        self._result: YabLocResult

        self.__sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb,
            100,
        )

    def diagnostics_cb(self, msg: DiagnosticArray) -> None:
        diag_status = msg.status[0]
        if diag_status.name != TARGET_DIAG_NAME:
            return
        self._result.set_frame(diag_status)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluator:
    return YabLocEvaluator("yabloc_evaluator")


if __name__ == "__main__":
    main()
