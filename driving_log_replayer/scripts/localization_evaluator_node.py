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

from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass
from dataclasses import field
from functools import singledispatchmethod
import statistics
from typing import ClassVar

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


def calc_pose_lateral_distance(ndt_pose: PoseStamped, ekf_pose: Odometry) -> float:
    base_point = ndt_pose.pose.position
    _, _, yaw = euler_from_quaternion(
        [
            ndt_pose.pose.orientation.x,
            ndt_pose.pose.orientation.y,
            ndt_pose.pose.orientation.z,
            ndt_pose.pose.orientation.w,
        ],
    )
    base_unit_vec = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    dx = ekf_pose.pose.pose.position.x - base_point.x
    dy = ekf_pose.pose.pose.position.y - base_point.y
    diff_vec = np.array([dx, dy, 0.0])
    cross_vec = np.cross(base_unit_vec, diff_vec)
    return cross_vec[2]


def calc_pose_horizontal_distance(ndt_pose: PoseStamped, ekf_pose: Odometry) -> float:
    ndt_x = ndt_pose.pose.position.x
    ndt_y = ndt_pose.pose.position.y
    ekf_x = ekf_pose.pose.pose.position.x
    ekf_y = ekf_pose.pose.pose.position.y
    return np.sqrt(np.power(ndt_x - ekf_x, 2) + np.power(ndt_y - ekf_y, 2))


@dataclass
class AbilityResult(ABC):
    ability_name: ClassVar[str] = "Ability"
    condition: dict = field(default_factory=dict)
    total: int = 0
    summary: str = "NotTested"
    success: bool = True

    def success_str(self) -> str:
        return "Success" if self.success else "Fail"

    @abstractmethod
    def set_frame(self) -> dict:
        return {}


@dataclass
class ConvergenceResult(AbilityResult):
    ability_name: ClassVar[str] = "Convergence"
    passed: int = 0

    def set_frame(
        self,
        msg: PoseStamped,
        map_to_baselink: dict,
        ekf_pose: Odometry,
        exe_time: Float32Stamped,
        iteration_num: Int32Stamped,
    ) -> tuple[dict, Float64]:
        self.total += 1
        lateral_dist = calc_pose_lateral_distance(msg, ekf_pose)
        horizontal_dist = calc_pose_horizontal_distance(msg, ekf_pose)

        msg_lateral_dist = Float64()
        msg_lateral_dist.data = lateral_dist

        exe_time_ms = exe_time.data
        iteration = iteration_num.data

        if (
            abs(lateral_dist) <= self.condition["AllowableDistance"]
            and exe_time_ms <= self.condition["AllowableExeTimeMs"]
            and iteration <= self.condition["AllowableIterationNum"]
        ):
            self.passed += 1

        current_rate = self.passed / self.total * 100.0
        self.success = current_rate >= self.condition["PassRate"]
        self.summary = f"{self.ability_name} ({self.success_str()}): {self.passed} / {self.total} -> {current_rate:.2f}%"

        return {
            "Ego": {"TransformStamped": map_to_baselink},
            "Convergence": {
                "Result": self.success_str(),
                "Info": [
                    {
                        "LateralDistance": lateral_dist,
                        "HorizontalDistance": horizontal_dist,
                        "ExeTimeMs": exe_time_ms,
                        "IterationNum": iteration,
                    },
                ],
            },
        }, msg_lateral_dist


@dataclass
class ReliabilityResult(AbilityResult):
    ability_name: ClassVar[str] = "Reliability"
    ng_seq: int = 0
    received_data: list[float] = field(default_factory=list)

    def set_frame(
        self,
        msg: Float32Stamped,
        map_to_baselink: dict,
        reference: Float32Stamped,
    ) -> dict:
        self.total += 1
        self.received_data.append(msg.data)

        # If the likelihood is lower than AllowableLikelihood for NGCount consecutive times, it is assumed to be a failure.
        if self.success:
            # Update nq_seq only while reliability.result is true
            if msg.data >= self.condition["AllowableLikelihood"]:
                self.ng_seq = 0
            else:
                self.ng_seq += 1
        self.success = bool(
            self.ng_seq < self.condition["NGCount"],
        )

        self.summary = f"{self.ability_name} ({self.success_str()}): {self.condition['Method']} Sequential NG Count: {self.ng_seq} (Total Test: {self.total}, Average: {statistics.mean(self.received_data):.5f}, StdDev: {statistics.pstdev(self.received_data):.5f})"
        return {
            "Ego": {"TransformStamped": map_to_baselink},
            "Reliability": {
                "Result": self.success_str(),
                "Info": [
                    {
                        "Value": message_to_ordereddict(msg),
                        "Reference": message_to_ordereddict(reference),
                    },
                ],
            },
        }


@dataclass
class AvailabilityResult(AbilityResult):
    ability_name: ClassVar[str] = "NDT Availability"
    TARGET_DIAG_NAME: ClassVar[
        str
    ] = "/autoware/localization/node_alive_monitoring/topic_status/topic_state_monitor_ndt_scan_matcher_exe_time: localization_topic_status"
    ERROR_STATUS_LIST: ClassVar[list[str]] = ["Timeout", "NotReceived"]

    def set_frame(self, msg: DiagnosticArray) -> dict:
        # Check if the NDT is available. Note that it does NOT check topic rate itself, but just the availability of the topic
        for diag_status in msg.status:
            if diag_status.name != AvailabilityResult.TARGET_DIAG_NAME:
                continue
            values = {value.key: value.value for value in diag_status.values}
            # Here we assume that, once a node (e.g. ndt_scan_matcher) fails, it will not be relaunched automatically.
            # On the basis of this assumption, we only consider the latest diagnostics received.
            # Possible status are OK, Timeout, NotReceived, WarnRate, and ErrorRate
            if values["status"] in AvailabilityResult.ERROR_STATUS_LIST:
                self.success = False
                self.summary = f"{self.ability_name} ({self.success_str()}): NDT not available"
            else:
                self.success = True
                self.summary = f"{self.ability_name} ({self.success_str()}): NDT available"
        return {
            "Availability": {
                "Result": self.success_str(),
                "Info": [
                    {},
                ],
            },
        }


class LocalizationResult(ResultBase):
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__convergence = ConvergenceResult(condition=condition["Convergence"])
        self.__reliability = ReliabilityResult(condition=condition["Reliability"])
        self.__availability = AvailabilityResult()

    def update(self) -> None:
        summary_str = f"{self.__convergence.summary}, {self.__reliability.summary}, {self.__availability.summary}"
        if (
            self.__convergence.success
            and self.__reliability.success
            and self.__availability.success
        ):
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    @singledispatchmethod
    def set_frame(self) -> None:
        raise NotImplementedError

    @set_frame.register
    def set_reliability_frame(
        self,
        msg: Float32Stamped,
        map_to_baselink: dict,
        reference: Float32Stamped,
    ) -> None:
        self._frame = self.__reliability.set_frame(msg, map_to_baselink, reference)
        self.update()

    @set_frame.register
    def set_convergence_frame(
        self,
        msg: PoseStamped,
        map_to_baselink: dict,
        ekf_pose: Odometry,
        exe_time: Float32Stamped,
        iteration_num: Int32Stamped,
    ) -> Float64:
        self._frame, msg_lateral_dist = self.__convergence.set_frame(
            msg,
            map_to_baselink,
            ekf_pose,
            exe_time,
            iteration_num,
        )
        self.update()
        return msg_lateral_dist

    @set_frame.register
    def set_ndt_availability_frame(self, msg: DiagnosticArray) -> None:
        self._frame = self.__availability.set_frame(msg)
        self.update()


class LocalizationEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.check_scenario()
        self.__result = LocalizationResult(self._condition)

        self.__latest_ekf_pose: Odometry = Odometry()
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
                    f"Reliability Method {self.__reliability_method} is not defined.",
                )
                rclpy.shutdown()
        except KeyError:
            self.get_logger().error("Scenario format error")
            rclpy.shutdown()

    def ekf_pose_cb(self, msg: Odometry) -> None:
        self.__latest_ekf_pose = msg

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
        self.__result.set_frame(
            msg,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_nvtl,
        )
        self._result_writer.write_result(self.__result)

    def nvtl_cb(self, msg: Float32Stamped) -> None:
        self.__latest_nvtl = msg
        if self.__reliability_method != "NVTL":
            # evaluates when reliability_method is NVTL
            return
        map_to_baselink = self.lookup_transform(msg.stamp)
        self.__result.set_frame(
            msg,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_tp,
        )
        self._result_writer.write_result(self.__result)

    def pose_cb(self, msg: PoseStamped) -> None:
        map_to_baselink = self.lookup_transform(msg.header.stamp)
        msg_lateral_distance = self.__result.set_frame(
            msg,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            self.__latest_ekf_pose,
            self.__latest_exe_time,
            self.__latest_iteration_num,
        )
        self.__pub_lateral_distance.publish(msg_lateral_distance)  # TODO: add integration test
        self._result_writer.write_result(self.__result)

    def diagnostics_cb(self, msg: DiagnosticArray) -> None:
        self.__result.set_frame(msg)
        self._result_writer.write_result(self.__result)


@evaluator_main
def main() -> DLREvaluator:
    return LocalizationEvaluator("localization_evaluator")


if __name__ == "__main__":
    main()
