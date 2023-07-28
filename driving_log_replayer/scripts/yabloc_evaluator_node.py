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


import os
from typing import Dict

from autoware_adapi_v1_msgs.srv import InitializeLocalization
from diagnostic_msgs.msg import DiagnosticArray
from driving_log_replayer.node_common import set_initial_pose
from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tier4_localization_msgs.srv import PoseWithCovarianceStamped
import yaml


class YabLocResult(ResultBase):
    def __init__(self, condition: Dict):
        super().__init__()
        # availability
        self.__yabloc_availability_result = False
        self.__yabloc_availability_msg = "NotTested"

    def update(self):
        if self.__yabloc_availability_result:
            yabloc_availability_summary = f"YabLoc Availability (Passed): {self.__yabloc_availability_msg}"
        else:
            yabloc_availability_summary = f"YabLoc Availability (Failed): {self.__yabloc_availability_msg}"
        summary_str = f"{yabloc_availability_summary}"
        if (self.__yabloc_availability_result):
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def add_yabloc_availability_frame(self, msg: DiagnosticArray):
        for diag_status in msg.status:
            if (diag_status.name != "yabloc_monitor: yabloc_status"):
                continue
            values = {value.key: value.value for value in diag_status.values}
            self.__yabloc_availability_result = values["Availability"] == "OK"


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
        with open(scenario_path, "r") as scenario_file:
            self.__scenario_yaml_obj = yaml.safe_load(scenario_file)
        self.__result_json_path = os.path.expandvars(
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )

        self.__condition = self.__scenario_yaml_obj["Evaluation"]["Conditions"]

        self.__result = YabLocResult(self.__condition)

        self.__result_writer = ResultWriter(
            self.__result_json_path, self.get_clock(), self.__condition
        )

        _ = set_initial_pose(
            self.__scenario_yaml_obj["Evaluation"]["InitialPose"]
        )

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

        self.__sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb,
            1,
        )

    def diagnostics_cb(self, msg: DiagnosticArray):
        self.__result.add_yabloc_availability_frame(msg)
        self.__result_writer.write(self.__result)

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
