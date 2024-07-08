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


from re import fullmatch

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from example_interfaces.msg import Byte
from example_interfaces.msg import Float64

from log_evaluator.evaluator import DLREvaluator
from log_evaluator.evaluator import evaluator_main
from log_evaluator.performance_diag import PerformanceDiagResult
from log_evaluator.performance_diag import PerformanceDiagScenario

REGEX_VISIBILITY_DIAG_NAME = "dual_return_filter: /sensing/lidar/.*: visibility_validation"
BLOCKAGE_DIAG_NAME = "blockage_return_diag: /sensing/lidar/.*: blockage_validation"


def extract_lidar_name(diag_name: str) -> str:
    remove_prefix = diag_name.replace("blockage_return_diag: /sensing/lidar/", "")
    return remove_prefix.replace(": blockage_validation", "")


class PerformanceDiagEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, PerformanceDiagScenario, PerformanceDiagResult)
        self._scenario: PerformanceDiagScenario
        self._result: PerformanceDiagResult

        self.__pub_visibility_value = self.create_publisher(Float64, "visibility/value", 1)
        self.__pub_visibility_level = self.create_publisher(Byte, "visibility/level", 1)

        self.__pub_blockage_ground_ratios = {}
        self.__pub_blockage_sky_ratios = {}
        self.__pub_blockage_levels = {}

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
            100,
        )

    def diag_cb(self, msg: DiagnosticArray) -> None:
        diag_status: DiagnosticStatus = msg.status[0]
        is_visibility = bool(fullmatch(REGEX_VISIBILITY_DIAG_NAME, diag_status.name))
        is_blockage = bool(fullmatch(BLOCKAGE_DIAG_NAME, diag_status.name))

        if not (is_visibility or is_blockage):
            return

        map_to_baselink = self.lookup_transform(msg.header.stamp)
        if is_visibility:
            msg_visibility_value, msg_visibility_level = self._result.set_visibility_frame(
                diag_status,
                DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            )
            if msg_visibility_value is not None:
                self.__pub_visibility_value.publish(msg_visibility_value)
            if msg_visibility_level is not None:
                self.__pub_visibility_level.publish(msg_visibility_level)
        if is_blockage:
            lidar_name = extract_lidar_name(diag_status.name)
            msg_blockage_sky_ratio, msg_blockage_ground_ratio, msg_blockage_level = (
                self._result.set_blockage_frame(
                    diag_status,
                    DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
                    lidar_name,
                )
            )
            if msg_blockage_sky_ratio is not None:
                self.__pub_blockage_sky_ratios[lidar_name].publish(msg_blockage_sky_ratio)
            if msg_blockage_ground_ratio is not None:
                self.__pub_blockage_ground_ratios[lidar_name].publish(msg_blockage_ground_ratio)
            if msg_blockage_level is not None:
                self.__pub_blockage_levels[lidar_name].publish(msg_blockage_level)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluator:
    return PerformanceDiagEvaluator("performance_diag_evaluator")


if __name__ == "__main__":
    main()
