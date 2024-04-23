#!/usr/bin/env python3

# Copyright (c) 2022 TIER IV.inc
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
from example_interfaces.msg import Byte
from example_interfaces.msg import Float64
from std_msgs.msg import Header

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.performance_diag import PerformanceDiagResult
from driving_log_replayer.performance_diag import PerformanceDiagScenario


class PerformanceDiagEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, PerformanceDiagScenario, PerformanceDiagResult)
        self._scenario: PerformanceDiagScenario

        self.__pub_visibility_value = self.create_publisher(Float64, "visibility/value", 1)
        self.__pub_visibility_level = self.create_publisher(Byte, "visibility/level", 1)

        self.__pub_blockage_ground_ratios = {}
        self.__pub_blockage_sky_ratios = {}
        self.__pub_blockage_levels = {}
        self.__diag_header_prev = Header()

        for k, v in self._scenario.Evaluation.Conditions.LiDAR.Blockage.items():
            if v.ScenarioType is not None:
                self.__pub_blockage_sky_ratios[k] = self.create_publisher(
                    Float64,
                    f"blockage/{k}/sky/ratio",
                    1,
                )
                self.__pub_blockage_ground_ratios[k] = self.create_publisher(
                    Float64,
                    f"blockage/{k}/ground/ratio",
                    1,
                )
                self.__pub_blockage_levels[k] = self.create_publisher(
                    Byte,
                    f"blockage/{k}/level",
                    1,
                )

        self.__sub_diag = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diag_cb,
            1,
        )

    def diag_cb(self, msg: DiagnosticArray) -> None:
        if msg.header == self.__diag_header_prev:
            return
        self.__diag_header_prev = msg.header
        map_to_baselink = self.lookup_transform(msg.header.stamp)
        (
            msg_visibility_value,
            msg_visibility_level,
            msg_blockage_sky_ratios,
            msg_blockage_ground_ratios,
            msg_blockage_levels,
        ) = self._result.set_frame(
            msg,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
        )
        if msg_visibility_value is not None:
            self.__pub_visibility_value.publish(msg_visibility_value)
        if msg_visibility_level is not None:
            self.__pub_visibility_level.publish(msg_visibility_level)
        for k, v in msg_blockage_sky_ratios.items():
            if v is not None:
                self.__pub_blockage_sky_ratios[k].publish(v)
        for k, v in msg_blockage_ground_ratios.items():
            if v is not None:
                self.__pub_blockage_ground_ratios[k].publish(v)
        for k, v in msg_blockage_levels.items():
            if v is not None:
                self.__pub_blockage_levels[k].publish(v)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluator:
    return PerformanceDiagEvaluator("performance_diag_evaluator")


if __name__ == "__main__":
    main()
