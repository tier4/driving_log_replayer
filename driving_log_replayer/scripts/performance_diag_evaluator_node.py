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


import re
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from example_interfaces.msg import Byte
from example_interfaces.msg import Float64
from std_msgs.msg import Header

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.result import ResultBase

REGEX_VISIBILITY_DIAG_NAME = "/autoware/sensing/lidar/performance_monitoring/visibility/.*"
BLOCKAGE_DIAG_BASE_NAME = (
    "/autoware/sensing/lidar/performance_monitoring/blockage/blockage_return_diag:  sensing lidar"
)
REGEX_BLOCKAGE_DIAG_NAME = BLOCKAGE_DIAG_BASE_NAME + ".*"


def get_diag_value(diag_status: DiagnosticStatus, key_name: str) -> str:
    for key_value in diag_status.values:
        if key_value.key == key_name:
            return key_value.value
    return ""


def trim_lidar_name(diag_name: str) -> str:
    remove_prefix = diag_name.replace(f"{BLOCKAGE_DIAG_BASE_NAME} ", "")
    return remove_prefix.replace(": blockage_validation", "")


class PerformanceDiagResult(ResultBase):
    VALID_VALUE_THRESHOLD = 0.0

    def __init__(self, condition: Dict) -> None:
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
        if self.__visibility_result:
            visibility_summary = f"Visibility (Passed): {self.__visibility_msg}"
        else:
            visibility_summary = f"Visibility (Failed): {self.__visibility_msg}"
        if self.__blockage_result:
            blockage_summary = f"Blockage (Passed): {self.__blockage_msg}"
        else:
            blockage_summary = f"Blockage (Failed): {self.__blockage_msg}"
        summary_str = f"{visibility_summary}, {blockage_summary}"
        if self.__visibility_result and self.__blockage_result:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = True
            self._summary = f"Failed: {summary_str}"

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
            if float_value >= PerformanceDiagResult.VALID_VALUE_THRESHOLD:
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
            if (
                float_sky_ratio >= PerformanceDiagResult.VALID_VALUE_THRESHOLD
                and float_ground_ratio >= PerformanceDiagResult.VALID_VALUE_THRESHOLD
            ):
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
            self.__blockage_msg += f"{lidar_name}: {self.__blockage_lidar_success[lidar_name]} / {self.__blockage_total} "
            # 1個でもFalseが入ってたらlidar試験全体がfalse
            if not v:
                self.__blockage_result = False

    def set_frame(
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


class PerformanceDiagEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.check_scenario()
        self.__result = PerformanceDiagResult(self._condition)

        self.__pub_visibility_value = self.create_publisher(Float64, "visibility/value", 1)
        self.__pub_visibility_level = self.create_publisher(Byte, "visibility/level", 1)

        self.__pub_blockage_ground_ratios = {}
        self.__pub_blockage_sky_ratios = {}
        self.__pub_blockage_levels = {}
        self.__diag_header_prev = Header()

        for k, v in self._condition["LiDAR"]["Blockage"].items():
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

        self.__sub_diag = self.create_subscription(
            DiagnosticArray,
            "/diagnostics_agg",
            self.diag_cb,
            1,
        )

    def check_scenario(self) -> None:
        pass

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
        ) = self.__result.set_frame(
            msg, DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink)
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
        self._result_writer.write(self.__result)


@evaluator_main
def main() -> DLREvaluator:
    return PerformanceDiagEvaluator("performance_diag_evaluator")


if __name__ == "__main__":
    main()
