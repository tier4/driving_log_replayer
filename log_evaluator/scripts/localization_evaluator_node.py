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
from diagnostic_msgs.msg import DiagnosticStatus
from example_interfaces.msg import Float64
from geometry_msgs.msg import PoseStamped
from tier4_debug_msgs.msg import Float32Stamped
from tier4_debug_msgs.msg import Int32Stamped

from log_evaluator.evaluator import DLREvaluator
from log_evaluator.evaluator import evaluator_main
from log_evaluator.localization import calc_pose_horizontal_distance
from log_evaluator.localization import calc_pose_lateral_distance
from log_evaluator.localization import LocalizationResult
from log_evaluator.localization import LocalizationScenario

TARGET_DIAG_NAME = "topic_state_monitor_ndt_scan_matcher_exe_time: localization_topic_status"


class LocalizationEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, LocalizationScenario, LocalizationResult)
        self._scenario: LocalizationScenario
        self._result: LocalizationResult

        self.__reliability_method = self._scenario.Evaluation.Conditions.Reliability.Method

        self.__latest_exe_time: Float32Stamped = Float32Stamped()
        self.__latest_iteration_num: Int32Stamped = Int32Stamped()
        self.__latest_tp: Float32Stamped = Float32Stamped()
        self.__latest_nvtl: Float32Stamped = Float32Stamped()

        self.__pub_lateral_distance = self.create_publisher(
            Float64,
            "localization/lateral_distance",
            1,
        )
        self.__sub_tp = self.create_subscription(
            Float32Stamped,
            "/localization/pose_estimator/transform_probability",
            self.tp_cb,
            1,
        )
        self.__sub_nvtl = self.create_subscription(
            Float32Stamped,
            "/localization/pose_estimator/nearest_voxel_transformation_likelihood",
            self.nvtl_cb,
            1,
        )
        self.__sub_relative_pose = self.create_subscription(
            PoseStamped,
            "/localization/pose_estimator/initial_to_result_relative_pose",
            self.relative_pose_cb,
            1,
        )
        self.__sub_exe_time = self.create_subscription(
            Float32Stamped,
            "/localization/pose_estimator/exe_time_ms",
            self.exe_time_cb,
            1,
        )
        self.__sub_iteration_num = self.create_subscription(
            Int32Stamped,
            "/localization/pose_estimator/iteration_num",
            self.iteration_num_cb,
            1,
        )
        self.__sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb,
            100,
        )

    def exe_time_cb(self, msg: Float32Stamped) -> None:
        self.__latest_exe_time = msg

    def iteration_num_cb(self, msg: Int32Stamped) -> None:
        self.__latest_iteration_num = msg

    def tp_cb(self, msg: Float32Stamped) -> None:
        self.__latest_tp = msg
        if self.__reliability_method != "TP":
            # evaluates when reliability_method is TP
            return
        map_to_baselink = self.lookup_transform(msg.stamp)
        self._result.set_frame(
            msg,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_nvtl,
        )
        self._result_writer.write_result(self._result)

    def nvtl_cb(self, msg: Float32Stamped) -> None:
        self.__latest_nvtl = msg
        if self.__reliability_method != "NVTL":
            # evaluates when reliability_method is NVTL
            return
        map_to_baselink = self.lookup_transform(msg.stamp)
        self._result.set_frame(
            msg,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_tp,
        )
        self._result_writer.write_result(self._result)

    def relative_pose_cb(self, msg: PoseStamped) -> None:
        map_to_baselink = self.lookup_transform(msg.header.stamp)
        msg_lateral_distance = self._result.set_frame(
            calc_pose_lateral_distance(msg),
            calc_pose_horizontal_distance(msg),
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_exe_time,
            self.__latest_iteration_num,
        )
        self.__pub_lateral_distance.publish(msg_lateral_distance)  # TODO: add integration test
        self._result_writer.write_result(self._result)

    def diagnostics_cb(self, msg: DiagnosticArray) -> None:
        if len(msg.status) == 0:
            return
        diag_status: DiagnosticStatus = msg.status[0]
        if diag_status.name != TARGET_DIAG_NAME:
            return
        self._result.set_frame(diag_status)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluator:
    return LocalizationEvaluator("localization_evaluator")


if __name__ == "__main__":
    main()
