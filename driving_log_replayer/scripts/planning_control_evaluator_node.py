#!/usr/bin/env python3

# Copyright (c) 2024 TIER IV.inc
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
from driving_log_replayer.planning_control import PlanningControlResult
from driving_log_replayer.planning_control import PlanningControlScenario


class PlanningControlEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, PlanningControlScenario, PlanningControlResult)
        self._scenario: PlanningControlScenario
        self._result: PlanningControlResult

        self.__sub_planning_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/planning/planning_evaluator/metrics",
            lambda msg, module_type="planning": self.diagnostics_cb(msg, module_type),
            1,
        )

        self.__sub_control_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/control/control_evaluator/metrics",
            lambda msg, module_type="control": self.diagnostics_cb(msg, module_type),
            1,
        )

    def diagnostics_cb(self, msg: DiagnosticArray, module: str) -> None:
        self._result.set_frame(msg, module)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluator:
    return PlanningControlEvaluator("planning_control_evaluator")


if __name__ == "__main__":
    main()
