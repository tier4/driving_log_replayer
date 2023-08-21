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
import statistics
from typing import Dict
from typing import TYPE_CHECKING

from autoware_adapi_v1_msgs.srv import InitializeLocalization
from diagnostic_msgs.msg import DiagnosticArray
from example_interfaces.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.time import Time
from rosidl_runtime_py import message_to_ordereddict
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from tf_transformations import euler_from_quaternion
from tier4_debug_msgs.msg import Float32Stamped
from tier4_debug_msgs.msg import Int32Stamped
from tier4_localization_msgs.srv import PoseWithCovarianceStamped
import yaml

from driving_log_replayer.node_common import set_initial_pose
from driving_log_replayer.node_common import transform_stamped_with_euler_angle
from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter

if TYPE_CHECKING:
    from autoware_adapi_v1_msgs.msg import ResponseStatus


def calc_pose_lateral_distance(ndt_pose: PoseStamped, ekf_pose: Odometry):
    base_point = ndt_pose.pose.position
    _, _, yaw = euler_from_quaternion(
        [
            ndt_pose.pose.orientation.x,
            ndt_pose.pose.orientation.y,
            ndt_pose.pose.orientation.z,
            ndt_pose.pose.orientation.w,
        ]
    )
    base_unit_vec = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    dx = ekf_pose.pose.pose.position.x - base_point.x
    dy = ekf_pose.pose.pose.position.y - base_point.y
    diff_vec = np.array([dx, dy, 0.0])
    cross_vec = np.cross(base_unit_vec, diff_vec)
    return cross_vec[2]


def calc_pose_horizontal_distance(ndt_pose: PoseStamped, ekf_pose: Odometry):
    ndt_x = ndt_pose.pose.position.x
    ndt_y = ndt_pose.pose.position.y
    ekf_x = ekf_pose.pose.pose.position.x
    ekf_y = ekf_pose.pose.pose.position.y
    return np.sqrt(np.power(ndt_x - ekf_x, 2) + np.power(ndt_y - ekf_y, 2))


class LocalizationResult(ResultBase):
    def __init__(self, condition: Dict):
        super().__init__()
        # convergence
        self.__convergence_condition: Dict = condition["Convergence"]
        self.__convergence_total = 0
        self.__convergence_success = 0
        self.__convergence_msg = "NotTested"
        self.__convergence_result = True
        # reliability
        self.__reliability_condition: Dict = condition["Reliability"]
        self.__reliability_ng_seq = 0
        self.__reliability_total = 0
        self.__reliability_msg = "NotTested"
        self.__reliability_result = True
        self.__reliability_list = []
        # availability
        self.__ndt_availability_error_status_list = ["Timeout", "NotReceived"]
        self.__ndt_availability_msg = "NotTested"
        self.__ndt_availability_result = True

    def update(self):
        if self.__convergence_result:
            convergence_summary = f"Convergence (Passed): {self.__convergence_msg}"
        else:
            convergence_summary = f"Convergence (Failed): {self.__convergence_msg}"
        if self.__reliability_result:
            reliability_summary = f"Reliability (Passed): {self.__reliability_msg}"
        else:
            reliability_summary = f"Reliability (Failed): {self.__reliability_msg}"
        if self.__ndt_availability_result:
            ndt_availability_summary = f"NDT Availability (Passed): {self.__ndt_availability_msg}"
        else:
            ndt_availability_summary = f"NDT Availability (Failed): {self.__ndt_availability_msg}"
        summary_str = f"{convergence_summary}, {reliability_summary}, {ndt_availability_summary}"
        if (
            self.__convergence_result
            and self.__reliability_result
            and self.__ndt_availability_result
        ):
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def add_reliability_frame(
        self, msg: Float32Stamped, map_to_baselink: Dict, reference: Float32Stamped
    ):
        self.__reliability_total += 1
        out_frame = {"Ego": {"TransformStamped": map_to_baselink}}

        reliability_value = msg.data
        self.__reliability_list.append(reliability_value)

        if self.__reliability_result:
            if reliability_value >= self.__reliability_condition["AllowableLikelihood"]:
                self.__reliability_ng_seq = 0
            else:
                self.__reliability_ng_seq += 1
        self.__reliability_result = (
            True if self.__reliability_ng_seq < self.__reliability_condition["NGCount"] else False
        )
        success = "Success" if self.__reliability_result else "Fail"

        out_frame["Reliability"] = {
            "Result": success,
            "Info": [
                {
                    "Value": message_to_ordereddict(msg),
                    "Reference": message_to_ordereddict(reference),
                }
            ],
        }
        self.__reliability_msg = f"{self.__reliability_condition['Method']} Sequential NG Count: {self.__reliability_ng_seq} (Total Test: {self.__reliability_total}, Average: {statistics.mean(self.__reliability_list):.5f}, StdDev: {statistics.pstdev(self.__reliability_list):.5f})"
        self._frame = out_frame
        self.update()

    def add_convergence_frame(
        self,
        msg: PoseStamped,
        map_to_baselink: Dict,
        ekf_pose: Odometry,
        exe_time: Float32Stamped,
        iteration_num: Int32Stamped,
    ) -> Float64:
        self.__convergence_total += 1
        out_frame = {"Ego": {"TransformStamped": map_to_baselink}}
        lateral_dist = calc_pose_lateral_distance(msg, ekf_pose)
        horizontal_dist = calc_pose_horizontal_distance(msg, ekf_pose)

        msg_lateral_dist = Float64()
        msg_lateral_dist.data = lateral_dist

        exe_time_ms = exe_time.data
        iteration = iteration_num.data

        if (
            abs(lateral_dist) <= self.__convergence_condition["AllowableDistance"]
            and exe_time_ms <= self.__convergence_condition["AllowableExeTimeMs"]
            and iteration <= self.__convergence_condition["AllowableIterationNum"]
        ):
            self.__convergence_success += 1

        current_rate = self.__convergence_success / self.__convergence_total * 100.0
        self.__convergence_result = current_rate >= self.__convergence_condition["PassRate"]
        self.__convergence_msg = (
            f"{self.__convergence_success} / {self.__convergence_total} -> {current_rate:.2f}%"
        )

        out_frame["Convergence"] = {
            "Result": "Success" if self.__convergence_result else "Fail",
            "Info": [
                {
                    "LateralDistance": lateral_dist,
                    "HorizontalDistance": horizontal_dist,
                    "ExeTimeMs": exe_time_ms,
                    "IterationNum": iteration,
                }
            ],
        }
        self._frame = out_frame
        self.update()
        return msg_lateral_dist

    def add_ndt_availability_frame(self, msg: DiagnosticArray):
        # Check if the NDT is available. Note that it does NOT check topic rate itself, but just the availability of the topic
        for diag_status in msg.status:
            if (
                diag_status.name
                != "/autoware/localization/node_alive_monitoring/topic_status/topic_state_monitor_ndt_scan_matcher_exe_time: localization_topic_status"
            ):
                continue
            values = {value.key: value.value for value in diag_status.values}
            # Here we assume that, once a node (e.g. ndt_scan_matcher) fails, it will not be relaunched automatically.
            # On the basis of this assumption, we only consider the latest diagnostics received.
            # Possible status are OK, Timeout, NotReceived, WarnRate, and ErrorRate
            if values["status"] in self.__ndt_availability_error_status_list:
                self.__ndt_availability_msg = "NDT not available"
                self.__ndt_availability_result = False
            else:
                self.__ndt_availability_msg = "NDT available"
                self.__ndt_availability_result = True
        self.update()


class LocalizationEvaluator(Node):
    def __init__(self):
        super().__init__("localization_evaluator")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("result_json_path", "")

        self.__timer_group = MutuallyExclusiveCallbackGroup()
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self, spin_thread=True)

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

        self.__reliability_method = self.__condition["Reliability"]["Method"]
        if self.__reliability_method not in ["TP", "NVTL"]:
            self.get_logger().error(
                f"Reliability Method {self.__reliability_method} is not defined."
            )
            rclpy.shutdown()

        self.__result = LocalizationResult(self.__condition)

        self.__result_writer = ResultWriter(
            self.__result_json_path, self.get_clock(), self.__condition
        )

        self.__initial_pose = set_initial_pose(
            self.__scenario_yaml_obj["Evaluation"]["InitialPose"]
        )
        self.__initial_pose_running = False
        self.__initial_pose_success = False

        self.__current_time = Time().to_msg()
        self.__prev_time = Time().to_msg()
        self.__counter = 0

        self.__pub_lateral_distance = self.create_publisher(
            Float64, "localization/lateral_distance", 1
        )

        self.__latest_ekf_pose: Odometry = Odometry()
        self.__latest_exe_time: Float32Stamped = Float32Stamped()
        self.__latest_iteration_num: Int32Stamped = Int32Stamped()
        self.__latest_tp: Float32Stamped = Float32Stamped()
        self.__latest_nvtl: Float32Stamped = Float32Stamped()

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
        self.__sub_pose = self.create_subscription(
            PoseStamped,
            "/localization/pose_estimator/pose",
            self.pose_cb,
            1,
        )
        self.__sub_ekf_pose = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.ekf_pose_cb,
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
        self.__sub_diagnostics_agg = self.create_subscription(
            DiagnosticArray,
            "/diagnostics_agg",
            self.diagnostics_cb,
            1,
        )

    def ekf_pose_cb(self, msg: Odometry):
        self.__latest_ekf_pose = msg

    def exe_time_cb(self, msg: Float32Stamped):
        self.__latest_exe_time = msg

    def iteration_num_cb(self, msg: Int32Stamped):
        self.__latest_iteration_num = msg

    def tp_cb(self, msg: Float32Stamped):
        self.__latest_tp = msg
        if not self.__reliability_method == "TP":
            # evaluates when reliability_method is TP
            return
        try:
            map_to_baselink = self.__tf_buffer.lookup_transform(
                "map", "base_link", msg.stamp, Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            map_to_baselink = TransformStamped()
        self.__result.add_reliability_frame(
            msg, transform_stamped_with_euler_angle(map_to_baselink), self.__latest_nvtl
        )
        self.__result_writer.write(self.__result)

    def nvtl_cb(self, msg: Float32Stamped):
        self.__latest_nvtl = msg
        if not self.__reliability_method == "NVTL":
            # evaluates when reliability_method is NVTL
            return
        try:
            map_to_baselink = self.__tf_buffer.lookup_transform(
                "map", "base_link", msg.stamp, Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            map_to_baselink = TransformStamped()
        self.__result.add_reliability_frame(
            msg, transform_stamped_with_euler_angle(map_to_baselink), self.__latest_tp
        )
        self.__result_writer.write(self.__result)

    def pose_cb(self, msg: PoseStamped):
        try:
            map_to_baselink = self.__tf_buffer.lookup_transform(
                "map", "base_link", msg.header.stamp, Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            map_to_baselink = TransformStamped()
        msg_lateral_distance = self.__result.add_convergence_frame(
            msg,
            transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_ekf_pose,
            self.__latest_exe_time,
            self.__latest_iteration_num,
        )
        self.__pub_lateral_distance.publish(msg_lateral_distance)
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
                # result.pose_with_covarianceに補正済みデータが入っている
                # 補正済みデータでinitialposeを投げる
                # debug result.pose_with_covariance
                # self.get_logger().error(
                #     f"corrected_pose_with_covariance.pose.position.x: {result.pose_with_covariance.pose.pose.position.x}"
                # )
                # self.get_logger().error(
                #     f"corrected_pose_with_covariance.pose.position.y: {result.pose_with_covariance.pose.pose.position.y}"
                # )
                # self.get_logger().error(
                #     f"corrected_pose_with_covariance.pose.position.z: {result.pose_with_covariance.pose.pose.position.z}"
                # )
                # self.get_logger().error(
                #     f"corrected_pose_with_covariance.pose.orientation.x: {result.pose_with_covariance.pose.pose.orientation.x}"
                # )
                # self.get_logger().error(
                #     f"corrected_pose_with_covariance.pose.orientation.y: {result.pose_with_covariance.pose.pose.orientation.y}"
                # )
                # self.get_logger().error(
                #     f"corrected_pose_with_covariance.pose.orientation.z: {result.pose_with_covariance.pose.pose.orientation.z}"
                # )
                # self.get_logger().error(
                #     f"corrected_pose_with_covariance.pose.orientation.w: {result.pose_with_covariance.pose.pose.orientation.w}"
                # )
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

    def diagnostics_cb(self, msg: DiagnosticArray):
        self.__result.add_ndt_availability_frame(msg)
        self.__result_writer.write(self.__result)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    localization_evaluator = LocalizationEvaluator()
    executor.add_node(localization_evaluator)
    executor.spin()
    localization_evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
