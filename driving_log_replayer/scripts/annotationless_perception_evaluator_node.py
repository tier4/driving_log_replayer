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

from driving_log_replayer.annotationless_perception import AnnotationlessPerceptionResult
from driving_log_replayer.annotationless_perception import AnnotationlessPerceptionScenario
from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main


class AnnotationlessPerceptionEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, AnnotationlessPerceptionScenario, AnnotationlessPerceptionResult)
        self._scenario: AnnotationlessPerceptionScenario
        # update condition using launch argument
        self.declare_parameter("annotationless_threshold_file", "")
        self._scenario.Evaluation.Conditions.set_threshold_from_file(
            self.get_parameter("annotationless_threshold_file").get_parameter_value().string_value,
        )
        self.declare_parameter("annotationless_pass_range", "")
        self._scenario.Evaluation.Conditions.set_pass_range(
            self.get_parameter("annotationless_pass_range").get_parameter_value().string_value,
        )
        self._result_writer.write_condition(self._scenario.Evaluation.Conditions, updated=True)

        self.__sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostic/perception_online_evaluator/metrics",
            self.diagnostics_cb,
            1,
        )

    def diagnostics_cb(self, msg: DiagnosticArray) -> None:
        self._result.set_frame(msg)
        self._result_writer.write_result(self._result)

    def timer_cb(self) -> None:
        super().timer_cb(register_shutdown_func=self.write_metrics)

    def write_metrics(self) -> None:
        self._result.set_final_metrics()
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluator:
    return AnnotationlessPerceptionEvaluator("annotationless_evaluator")


if __name__ == "__main__":
    main()
