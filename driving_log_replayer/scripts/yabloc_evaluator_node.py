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

from autoware_adapi_v1_msgs.srv import InitializeLocalization
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tier4_localization_msgs.srv import PoseWithCovarianceStamped
import yaml

from driving_log_replayer.node_common import set_initial_pose
from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter

if TYPE_CHECKING:
    from autoware_adapi_v1_msgs.msg import ResponseStatus


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

    def add_yabloc_availability_frame(self, msg: DiagnosticArray):
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


class YabLocEvaluator(Node):
    def __init__(self):
        super().__init__("yabloc_evaluator")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("result_json_path", "")

        self.__timer_group = MutuallyExclusiveCallbackGroup()

        scenario_path = os.path.expandvars(
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )
        self.__scenario_yaml_obj = None
        with open(scenario_path) as scenario_file:
            self.__scenario_yaml_obj = yaml.safe_load(scenario_file)
        self.__result_json_path = os.path.expandvars(
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )

        self.__result = YabLocResult()

        self.__result_writer = ResultWriter(self.__result_json_path, self.get_clock(), {})

        self.__initial_pose = set_initial_pose(
            self.__scenario_yaml_obj["Evaluation"]["InitialPose"]
        )
        self.__initial_pose_running = False
        self.__initial_pose_success = False

        self.__current_time = Time().to_msg()
        self.__prev_time = Time().to_msg()
        self.__counter = 0

        # service client
        self.__initial_pose_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )
        self.__map_fit_client = self.create_client(
            PoseWithCovarianceStamped, "/map/map_height_fitter/service"
        )
        while not self.__initial_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("initial pose service not available, waiting again...")
        while not self.__map_fit_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("map height fitter service not available, waiting again...")

        self.__timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self.__timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer

        self.__sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb,
            1,
        )

    def diagnostics_cb(self, msg: DiagnosticArray):
        self.__result.add_yabloc_availability_frame(msg)
        self.__result_writer.write(self.__result)

    def timer_cb(self) -> None:
        self.__current_time = self.get_clock().now().to_msg()
        # self.get_logger().error(f"time: {self.__current_time.sec}.{self.__current_time.nanosec}")
        if self.__current_time.sec > 0:
            if (
                self.__initial_pose is not None
                and not self.__initial_pose_success
                and not self.__initial_pose_running
            ):
                self.get_logger().info(
                    f"call initial_pose time: {self.__current_time.sec}.{self.__current_time.nanosec}"
                )
                self.__initial_pose_running = True
                self.__initial_pose.header.stamp = self.__current_time
                future_map_fit = self.__map_fit_client.call_async(
                    PoseWithCovarianceStamped.Request(pose_with_covariance=self.__initial_pose)
                )
                future_map_fit.add_done_callback(self.map_fit_cb)
            if self.__current_time == self.__prev_time:
                self.__counter += 1
            else:
                self.__counter = 0
            self.__prev_time = self.__current_time
            if self.__counter >= 5:
                self.__result_writer.close()
                rclpy.shutdown()

    def map_fit_cb(self, future):
        result = future.result()
        if result is not None:
            if result.success:
                future_init_pose = self.__initial_pose_client.call_async(
                    InitializeLocalization.Request(pose=[result.pose_with_covariance])
                )
                future_init_pose.add_done_callback(self.initial_pose_cb)
            else:
                # free self.__initial_pose_running when the service call fails
                self.__initial_pose_running = False
                self.get_logger().warn("map_height_height service result is fail")
        else:
            # free self.__initial_pose_running when the service call fails
            self.__initial_pose_running = False
            self.get_logger().error(f"Exception for service: {future.exception()}")

    def initial_pose_cb(self, future):
        result = future.result()
        if result is not None:
            res_status: ResponseStatus = result.status
            self.__initial_pose_success = res_status.success
            self.get_logger().info(
                f"initial_pose_success: {self.__initial_pose_success}"
            )  # debug msg
        else:
            self.get_logger().error(f"Exception for service: {future.exception()}")
        # free self.__initial_pose_running
        self.__initial_pose_running = False


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    localization_evaluator = YabLocEvaluator()
    executor.add_node(localization_evaluator)
    executor.spin()
    localization_evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
