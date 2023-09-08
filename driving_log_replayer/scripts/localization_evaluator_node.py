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


from functools import singledispatchmethod
import statistics
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray
from example_interfaces.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rosidl_runtime_py import message_to_ordereddict
from tf_transformations import euler_from_quaternion
from tier4_debug_msgs.msg import Float32Stamped
from tier4_debug_msgs.msg import Int32Stamped

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.result import ResultBase


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
    def __init__(self, condition: Dict) -> None:
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

    @singledispatchmethod
    def set_frame(self, msg):
        pass

    @set_frame.register
    def set_reliability_frame(
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
        self.__reliability_result = bool(
            self.__reliability_ng_seq < self.__reliability_condition["NGCount"]
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

    @set_frame.register
    def set_convergence_frame(
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

    @set_frame.register
    def set_ndt_availability_frame(self, msg: DiagnosticArray):
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


class LocalizationEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.check_scenario()

        self.__result = LocalizationResult(self._condition)

        self.__pub_lateral_distance = self.create_publisher(
            Float64, "localization/lateral_distance", 1
        )

        self.__latest_ekf_pose: Odometry = Odometry()
        self.__latest_exe_time: Float32Stamped = Float32Stamped()
        self.__latest_iteration_num: Int32Stamped = Int32Stamped()
        self.__latest_tp: Float32Stamped = Float32Stamped()
        self.__latest_nvtl: Float32Stamped = Float32Stamped()

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

    def check_scenario(self) -> None:
        try:
            self.__reliability_method = self._condition["Reliability"]["Method"]
            if self.__reliability_method not in ["TP", "NVTL"]:
                self.get_logger().error(
                    f"Reliability Method {self.__reliability_method} is not defined."
                )
                rclpy.shutdown()
        except KeyError:
            self.get_logger().error("Scenario format error")
            rclpy.shutdown()

    def ekf_pose_cb(self, msg: Odometry):
        self.__latest_ekf_pose = msg

    def exe_time_cb(self, msg: Float32Stamped):
        self.__latest_exe_time = msg

    def iteration_num_cb(self, msg: Int32Stamped):
        self.__latest_iteration_num = msg

    def tp_cb(self, msg: Float32Stamped):
        self.__latest_tp = msg
        if self.__reliability_method != "TP":
            # evaluates when reliability_method is TP
            return
        map_to_baselink = self.lookup_transform(msg.stamp)
        self.__result.set_frame(
            msg,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_nvtl,
        )
        self._result_writer.write(self.__result)

    def nvtl_cb(self, msg: Float32Stamped):
        self.__latest_nvtl = msg
        if self.__reliability_method != "NVTL":
            # evaluates when reliability_method is NVTL
            return
        map_to_baselink = self.lookup_transform(msg.stamp)
        self.__result.set_frame(
            msg, DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink), self.__latest_tp
        )
        self._result_writer.write(self.__result)

    def pose_cb(self, msg: PoseStamped):
        map_to_baselink = self.lookup_transform(msg.header.stamp)
        msg_lateral_distance = self.__result.set_frame(
            msg,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_ekf_pose,
            self.__latest_exe_time,
            self.__latest_iteration_num,
        )
        self.__pub_lateral_distance.publish(msg_lateral_distance)
        self._result_writer.write(self.__result)

    def diagnostics_cb(self, msg: DiagnosticArray):
        self.__result.set_frame(msg)
        self._result_writer.write(self.__result)


@evaluator_main
def main() -> DLREvaluator:
    return LocalizationEvaluator("localization_evaluator")


if __name__ == "__main__":
    main()
