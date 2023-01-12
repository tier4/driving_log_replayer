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
import re
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from autoware_adapi_v1_msgs.msg import ResponseStatus
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from driving_log_replayer.node_common import set_initial_pose
from driving_log_replayer.node_common import transform_stamped_with_euler_angle
from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter
from example_interfaces.msg import Byte
from example_interfaces.msg import Float64
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from tier4_localization_msgs.srv import PoseWithCovarianceStamped
import yaml

REGEX_VISIBILITY_DIAG_NAME = "/autoware/sensing/lidar/performance_monitoring/visibility/.*"
BLOCKAGE_DIAG_BASE_NAME = (
    "/autoware/sensing/lidar/performance_monitoring/blockage/blockage_return_diag:  sensing lidar"
)
REGEX_BLOCKAGE_DIAG_NAME = BLOCKAGE_DIAG_BASE_NAME + ".*"


def get_diag_value(diag_status: DiagnosticStatus, key_name: str) -> str:
    diag_value = ""
    for key_value in diag_status.values:
        if key_value.key == key_name:
            diag_value = key_value.value
    return diag_value


def trim_lidar_name(diag_name: str) -> str:
    remove_prefix = diag_name.replace(f"{BLOCKAGE_DIAG_BASE_NAME} ", "")
    remove_suffix = remove_prefix.replace(": blockage_validation", "")
    return remove_suffix


class PerformanceDiagResult(ResultBase):
    def __init__(self, condition: Dict):
        super().__init__()
        # visibility
        self.__visibility_condition: Dict = condition["LiDAR"]["Visibility"]
        self.__visibility_total = 0
        self.__visibility_success = 0
        self.__visibility_msg = "NotTested"
        self.__visibility_result = True
        # blockage
        self.__blockage_condition: Dict = condition["LiDAR"]["Blockage"]
        self.__blockage_total = 0
        self.__blockage_msg = "NotTested"
        self.__blockage_result = True
        self.__blockage_lidar_success = {}
        self.__blockage_lidar_result = {}
        for k, _ in self.__blockage_condition.items():
            self.__blockage_lidar_success[k] = 0
            self.__blockage_lidar_result[k] = True

    def update(self):
        summary_str = f"Visibility: {self.__visibility_msg} Blockage: {self.__blockage_msg}"
        if self.__visibility_result and self.__blockage_result:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        elif self.__visibility_result and not self.__blockage_result:
            self._success = False
            self._summary = f"BlockageError: {summary_str}"
        elif not self.__visibility_result and self.__blockage_result:
            self._success = False
            self._summary = f"VisibilityError: {summary_str}"
        elif not self.__visibility_result and not self.__blockage_result:
            self._success = False
            self._summary = f"VisibilityAndBlockageError: {summary_str}"

    def summarize_diag_agg(
        self, msg: DiagnosticArray
    ) -> Tuple[
        List[Dict],
        Optional[Float64],
        Optional[Byte],
        List[Dict],
        Dict[str, Optional[Float64]],
        Dict[str, Optional[Float64]],
    ]:
        visibility_results = []
        msg_visibility_value = None
        msg_visibility_level = None
        blockage_results = []
        msg_blockage_sky_ratios = {}
        msg_blockage_ground_ratios = {}
        msg_blockage_levels = {}
        self.__visibility_total += 1
        self.__blockage_total += 1
        for diag_status in msg.status:
            if re.fullmatch(REGEX_VISIBILITY_DIAG_NAME, diag_status.name):
                (
                    visibility_result,
                    msg_visibility_value,
                    msg_visibility_level,
                ) = self.summarize_visibility(diag_status)
                visibility_results.append(visibility_result)
            elif re.fullmatch(REGEX_BLOCKAGE_DIAG_NAME, diag_status.name):
                (
                    blockage_result,
                    msg_blockage_sky_ratio,
                    msg_blockage_ground_ratio,
                    msg_blockage_level,
                ) = self.summarize_blockage(diag_status)
                blockage_results.append(blockage_result)
                lidar_name = trim_lidar_name(diag_status.name)
                msg_blockage_sky_ratios[lidar_name] = msg_blockage_sky_ratio
                msg_blockage_ground_ratios[lidar_name] = msg_blockage_ground_ratio
                msg_blockage_levels[lidar_name] = msg_blockage_level
        return (
            visibility_results,
            msg_visibility_value,
            msg_visibility_level,
            blockage_results,
            msg_blockage_sky_ratios,
            msg_blockage_ground_ratios,
            msg_blockage_levels,
        )

    def summarize_visibility(
        self, diag_status: DiagnosticStatus
    ) -> Tuple[Dict, Optional[Float64], Optional[Byte]]:
        result = "Skipped"
        info = []
        scenario_type = self.__visibility_condition["ScenarioType"]
        msg_value = None
        msg_level = None
        if scenario_type is not None:
            self.__visibility_result = True
            visibility_value = get_diag_value(diag_status, "value")
            diag_level = diag_status.level
            if scenario_type == "TP":
                if diag_level == DiagnosticStatus.ERROR:
                    result = "Success"
                    self.__visibility_success += 1
                else:
                    result = "Fail"
                # visibilityは1個なので、TPかTPかで更新したら、全体結果についての判定も出来る
                if self.__visibility_success < self.__visibility_condition["PassFrameCount"]:
                    self.__visibility_result = False
            elif scenario_type == "FP":
                if diag_level == DiagnosticStatus.ERROR:
                    result = "Fail"
                else:
                    result = "Success"
                    self.__visibility_success += 1
                # visibilityは1個なので、TPかTPかで更新したら、全体結果についての判定も出来る
                if self.__visibility_success != self.__visibility_total:
                    self.__visibility_result = False
            info.append(
                {
                    "Level": int.from_bytes(diag_level, byteorder="little"),
                    "Visibility": visibility_value,
                }
            )
            self.__visibility_msg = (
                f"passed: {self.__visibility_success} / {self.__visibility_total}"
            )
            # publishするmsgを作る
            float_value = float(visibility_value)
            if float_value >= 0.0:
                msg_value = Float64()
                msg_value.data = float_value
                msg_level = Byte()
                msg_level.data = diag_level
        return {"Result": result, "Info": info}, msg_value, msg_level

    def summarize_blockage(
        self, diag_status: DiagnosticStatus
    ) -> Tuple[Dict, Optional[Float64], Optional[Float64], Optional[Byte]]:
        result = "Skipped"
        info = []
        lidar_name = trim_lidar_name(diag_status.name)
        info_dict = {"Name": lidar_name}
        scenario_type = self.__blockage_condition[lidar_name]["ScenarioType"]
        msg_sky_ratio = None
        msg_ground_ratio = None
        msg_level = None
        if scenario_type is not None:
            self.__blockage_lidar_result[lidar_name] = True
            ground_ratio = get_diag_value(diag_status, "ground_blockage_ratio")
            sky_ratio = get_diag_value(diag_status, "sky_blockage_ratio")
            diag_level = diag_status.level
            if scenario_type == "TP":
                if diag_level == DiagnosticStatus.ERROR:
                    # sky, ground, bothが一致しているか確認する
                    blockage_type = self.__blockage_condition[lidar_name]["BlockageType"]
                    if blockage_type in diag_status.message:
                        result = "Success"
                        self.__blockage_lidar_success[lidar_name] += 1
                    else:
                        result = "Fail"
                else:
                    result = "Fail"
                # blockageはlidarの数分ある。個別のlidarの結果の更新を行う
                if (
                    self.__blockage_lidar_success[lidar_name]
                    < self.__blockage_condition[lidar_name]["PassFrameCount"]
                ):
                    self.__blockage_lidar_result[lidar_name] = False
            elif scenario_type == "FP":
                if diag_level == DiagnosticStatus.ERROR:
                    result = "Fail"
                else:
                    result = "Success"
                    self.__blockage_lidar_success[lidar_name] += 1
                # blockageはlidarの数分ある。個別のlidarの結果の更新を行う
                if self.__blockage_lidar_success[lidar_name] != self.__blockage_total:
                    self.__blockage_lidar_result[lidar_name] = False
            info_dict["Level"] = int.from_bytes(diag_level, byteorder="little")
            info_dict["GroundBlockageRatio"] = ground_ratio
            info_dict["GroundBlockageCount"] = get_diag_value(diag_status, "ground_blockage_count")
            info_dict["SkyBlockageRatio"] = sky_ratio
            info_dict["SkyBlockageCount"] = get_diag_value(diag_status, "sky_blockage_count")
            # publishするmsgを作る
            float_sky_ratio = float(sky_ratio)
            float_ground_ratio = float(ground_ratio)
            if float_sky_ratio >= 0.0 and float_ground_ratio >= 0.0:
                msg_sky_ratio = Float64()
                msg_sky_ratio.data = float_sky_ratio
                msg_ground_ratio = Float64()
                msg_ground_ratio.data = float_ground_ratio
                msg_level = Byte()
                msg_level.data = diag_level
        info.append(info_dict)
        return {"Result": result, "Info": info}, msg_sky_ratio, msg_ground_ratio, msg_level

    def update_blockage(self):
        self.__blockage_result = True
        self.__blockage_msg = ""
        for lidar_name, v in self.__blockage_lidar_result.items():
            self.__blockage_msg += f"{lidar_name}: {self.__blockage_lidar_success[lidar_name]}/{self.__blockage_total} "
            # 1個でもFalseが入ってたらlidar試験全体がfalse
            if not v:
                self.__blockage_result = False

    def add_frame(
        self, msg: DiagnosticArray, map_to_baselink: Dict
    ) -> Tuple[
        Optional[Float64],
        Optional[Byte],
        Dict[str, Optional[Float64]],
        Dict[str, Optional[Float64]],
        Dict[str, Optional[Byte]],
    ]:
        out_frame = {"Ego": {"TransformStamped": map_to_baselink}}
        (
            out_frame["Visibility"],
            msg_visibility_value,
            msg_visibility_level,
            out_frame["Blockage"],
            msg_blockage_sky_ratios,
            msg_blockage_ground_ratios,
            msg_blockage_levels,
        ) = self.summarize_diag_agg(msg)
        # blockageは全てのlidarの結果が出揃ったところで判定
        self.update_blockage()
        self._frame = out_frame
        self.update()
        return (
            msg_visibility_value,
            msg_visibility_level,
            msg_blockage_sky_ratios,
            msg_blockage_ground_ratios,
            msg_blockage_levels,
        )


class PerformanceDiagEvaluator(Node):
    def __init__(self):
        super().__init__("performance_diag_evaluator")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("result_json_path", "")
        self.declare_parameter("localization", False)

        self.__timer_group = MutuallyExclusiveCallbackGroup()
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self, spin_thread=True)

        scenario_path = os.path.expandvars(
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )
        self.__launch_localization = (
            self.get_parameter("scenario_path").get_parameter_value().bool_value
        )
        self.__scenario_yaml_obj = None
        with open(scenario_path, "r") as scenario_file:
            self.__scenario_yaml_obj = yaml.safe_load(scenario_file)
        self.__result_json_path = os.path.expandvars(
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )

        self.__condition = self.__scenario_yaml_obj["Evaluation"]["Conditions"]
        self.__result = PerformanceDiagResult(self.__condition)

        self.__result_writer = ResultWriter(
            self.__result_json_path, self.get_clock(), self.__condition
        )

        self.__initial_pose = set_initial_pose(
            self.__scenario_yaml_obj["Evaluation"]["InitialPose"]
        )
        self.__initial_pose_success = False

        self.__pub_visibility_value = self.create_publisher(Float64, "visibility/value", 1)
        self.__pub_visibility_level = self.create_publisher(Byte, "visibility/level", 1)

        self.__pub_blockage_ground_ratios = {}
        self.__pub_blockage_sky_ratios = {}
        self.__pub_blockage_levels = {}

        for k, v in self.__condition["LiDAR"]["Blockage"].items():
            if v["ScenarioType"] is not None:
                self.__pub_blockage_sky_ratios[k] = self.create_publisher(
                    Float64, f"blockage/{k}/sky/ratio", 1
                )
                self.__pub_blockage_ground_ratios[k] = self.create_publisher(
                    Float64, f"blockage/{k}/ground/ratio", 1
                )
                self.__pub_blockage_levels[k] = self.create_publisher(
                    Byte, f"blockage/{k}/level", 1
                )

        self.__current_time = Time().to_msg()
        self.__prev_time = Time().to_msg()
        self.__diag_header_prev = Header()
        self.__counter = 0

        self.__timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self.__timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer

        self.__sub_diag = self.create_subscription(
            DiagnosticArray,
            "/diagnostics_agg",
            self.diag_cb,
            1,
        )
        # service client
        self.__initial_pose_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )
        self.__map_fit_client = self.create_client(
            PoseWithCovarianceStamped, "/localization/util/fit_map_height"
        )
        if self.__launch_localization:
            while not self.__initial_pose_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning("service not available, waiting again...")
            while not self.__map_fit_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning("service not available, waiting again...")

    def timer_cb(self) -> None:
        self.__current_time = self.get_clock().now().to_msg()
        # self.get_logger().error(f"time: {self.__current_time.sec}.{self.__current_time.nanosec}")
        if self.__current_time.sec > 0:
            if (
                self.__launch_localization
                and self.__initial_pose is not None
                and not self.__initial_pose_success
            ):
                self.__initial_pose.header.stamp = self.__current_time
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
                future_init_pose = self.__initial_pose_client.call_async(
                    InitializeLocalization.Request(pose=[result.pose_with_covariance])
                )
                future_init_pose.add_done_callback(self.initial_pose_cb)
        else:
            self.get_logger().error(f"Exception for service: {future.exception()}")

    def initial_pose_cb(self, future):
        result = future.result()
        if result is not None:
            res_status: ResponseStatus = result.status
            self.__initial_pose_success = res_status.success
        else:
            self.get_logger().error(f"Exception for service: {future.exception()}")

    def diag_cb(self, msg: DiagnosticArray) -> None:
        # self.get_logger().error(
        #     f"diag cb time: {self.__current_time.sec}.{self.__current_time.nanosec}"
        # )
        if msg.header == self.__diag_header_prev:
            return
        self.__diag_header_prev = msg.header
        try:
            map_to_baselink = self.__tf_buffer.lookup_transform(
                "map", "base_link", msg.header.stamp, Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            map_to_baselink = TransformStamped()
        (
            msg_visibility_value,
            msg_visibility_level,
            msg_blockage_sky_ratios,
            msg_blockage_ground_ratios,
            msg_blockage_levels,
        ) = self.__result.add_frame(msg, transform_stamped_with_euler_angle(map_to_baselink))
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
        self.__result_writer.write(self.__result)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    performance_diag_evaluator = PerformanceDiagEvaluator()
    executor.add_node(performance_diag_evaluator)
    executor.spin()
    performance_diag_evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
