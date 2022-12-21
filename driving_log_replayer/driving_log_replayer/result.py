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

import json
import os
import pickle
from typing import Dict

from rclpy.clock import Clock
from rclpy.clock import ClockType


class ResultBase:
    def __init__(self) -> None:
        self._success = False
        self._summary = "NoData"
        self._frame = {}

    def success(self) -> bool:
        return self._success

    def summary(self) -> str:
        return self._summary

    def frame(self) -> Dict:
        return self._frame


class ResultWriter:
    def __init__(self, result_json_path: str, ros_clock: Clock, condition: Dict):
        # 拡張子を書き換える
        result_file = (
            os.path.splitext(os.path.expandvars(result_json_path))[0] + ".jsonl"
        )
        self._result_file = open(result_file, "w")
        self._ros_clock = ros_clock
        self._system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.write_condition(condition)
        self.write_header()

    def close(self):
        self._result_file.close()

    def write_condition(self, condition: Dict):
        dict_condition = {"Condition": condition}
        str_condition = json.dumps(dict_condition) + "\n"
        self._result_file.write(str_condition)

    def write_header(self):
        system_time = self._system_clock.now()
        dict_header = {
            "Result": {"Success": False, "Summary": "NoData"},
            "Stamp": {"System": system_time.nanoseconds / pow(10, 9)},
            "Frame": {},
        }
        str_header = json.dumps(dict_header) + "\n"
        self._result_file.write(str_header)

    def write(self, result: ResultBase):
        system_time = self._system_clock.now()
        time_dict = {"System": system_time.nanoseconds / pow(10, 9)}
        if self._ros_clock.ros_time_is_active:
            ros_time = self._ros_clock.now()
            time_dict["ROS"] = ros_time.nanoseconds / pow(10, 9)
        else:
            time_dict["ROS"] = 0.0

        dict_record = {
            "Result": {"Success": result.success(), "Summary": result.summary()},
            "Stamp": time_dict,
            "Frame": result.frame(),
        }
        str_record = json.dumps(dict_record) + "\n"
        self._result_file.write(str_record)


class PickleWriter:
    def __init__(self, out_pkl_path: str):
        self._pickle_file = open(os.path.expandvars(out_pkl_path), "wb")

    def dump(self, write_object):
        pickle.dump(write_object, self._pickle_file)
        self.close()

    def close(self):
        self._pickle_file.close()
