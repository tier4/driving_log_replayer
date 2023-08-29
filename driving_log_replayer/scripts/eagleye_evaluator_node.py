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

import os
from typing import TYPE_CHECKING

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.node import Node
from tier4_localization_msgs.srv import PoseWithCovarianceStamped
import yaml

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter

if TYPE_CHECKING:
    from autoware_adapi_v1_msgs.msg import ResponseStatus


class EagleyeResult(ResultBase):
    def __init__(self):
        super().__init__()
        # availability
        self.__eagleye_availability_result = False
        self.__eagleye_availability_msg = "NotTested"

    def update(self):
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

    def add_eagleye_availability_frame(self, msg: DiagnosticArray):
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
    def __init__(self, name: str):
        super().__init__(name)

        self.__result = EagleyeResult()
        self.__result_writer = ResultWriter(self.__result_json_path, self.get_clock(), {})

        self._scenario_yaml_obj = None
        with open(self._scenario_path) as scenario_file:
            self._scenario_yaml_obj = yaml.safe_load(scenario_file)

        self._initial_pose = DLREvaluator.set_initial_pose(
            self._scenario_yaml_obj["Evaluation"]["InitialPose"]
        )
        self.start_initial_pose()

        self.__sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb,
            1,
        )

    def diagnostics_cb(self, msg: DiagnosticArray):
        self.__result.add_eagleye_availability_frame(msg)
        self.__result_writer.write(self.__result)

    def timer_cb(self) -> None:
        self._current_time = self.get_clock().now().to_msg()
        # self.get_logger().error(f"time: {self.__current_time.sec}.{self.__current_time.nanosec}")
        if self._current_time.sec > 0:
            if (
                self._initial_pose is not None
                and not self._initial_pose_success
                and not self._initial_pose_running
            ):
                self.get_logger().info(
                    f"call initial_pose time: {self._current_time.sec}.{self._current_time.nanosec}"
                )
                self._initial_pose_running = True
                self._initial_pose.header.stamp = self.__current_time
                future_map_fit = self.__map_fit_client.call_async(
                    PoseWithCovarianceStamped.Request(pose_with_covariance=self.__initial_pose)
                )
                future_map_fit.add_done_callback(self.map_fit_cb)
            if self._current_time == self.__prev_time:
                self._clock_stop_counter += 1
            else:
                self._clock_stop_counter = 0
            self._prev_time = self._current_time
            if self._clock_stop_counter >= 5:
                self._result_writer.close()
                rclpy.shutdown()


if __name__ == "__main__":
    evaluator = EagleyeEvaluator("eagleye_evaluator")
    evaluator.run()
