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
from os.path import expandvars
from pathlib import Path
import pickle
from typing import Any
from typing import ClassVar

from rclpy.clock import Clock
from rclpy.clock import ClockType
import simplejson as json


@dataclass
class TopicResult(ABC):
    name: ClassVar[str] = "This field should be overwritten"
    condition: dict = field(default_factory=dict)
    total: int = 0
    summary: str = "NotTested"
    success: bool = True

    def success_str(self) -> str:
        return "Success" if self.success else "Fail"

    @abstractmethod
    def set_frame(self) -> dict:
        return {}


class ResultBase(ABC):
    def __init__(self) -> None:
        self._success = False
        self._summary = "NoData"
        self._frame = {}

    @property
    def success(self) -> bool:
        return self._success

    @property
    def summary(self) -> str:
        return self._summary

    @property
    def frame(self) -> dict:
        return self._frame

    @abstractmethod
    def update(self) -> None:
        """Update success and summary."""

    @abstractmethod
    def set_frame(self) -> None:
        """Set the result of one frame from the subscribe ros message."""


class ResultWriter:
    def __init__(self, result_json_path: str, ros_clock: Clock, condition: dict) -> None:
        self._result_path = self.create_jsonl_path(result_json_path)
        self._result_file = self._result_path.open("w")
        self._ros_clock = ros_clock
        self._system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.write_line({"Condition": condition})
        self.write_line(self.get_header())

    @property
    def result_path(self) -> Path:
        return self._result_path

    def create_jsonl_path(self, result_json_path: str) -> Path:
        # For compatibility with previous versions.
        # If a json file name is passed, replace it with the filename + jsonl
        original_path = Path(expandvars(result_json_path))
        return original_path.parent.joinpath(original_path.stem + ".jsonl")

    def close(self) -> None:
        self._result_file.close()

    def delete_result_file(self) -> None:
        self._result_path.unlink()

    def write_line(self, write_obj: Any) -> None:
        str_record = json.dumps(write_obj, ignore_nan=True) + "\n"
        self._result_file.write(str_record)

    def write_result(self, result: ResultBase) -> None:
        self.write_line(self.get_result(result))

    def get_header(self) -> dict:
        system_time = self._system_clock.now()
        return {
            "Result": {"Success": False, "Summary": "NoData"},
            "Stamp": {"System": system_time.nanoseconds / pow(10, 9)},
            "Frame": {},
        }

    def get_result(self, result: ResultBase) -> dict:
        system_time = self._system_clock.now()
        time_dict = {"System": system_time.nanoseconds / pow(10, 9)}
        if self._ros_clock.ros_time_is_active:
            ros_time = self._ros_clock.now()
            time_dict["ROS"] = ros_time.nanoseconds / pow(10, 9)
        else:
            time_dict["ROS"] = 0.0

        return {
            "Result": {"Success": result.success, "Summary": result.summary},
            "Stamp": time_dict,
            "Frame": result.frame,
        }


class PickleWriter:
    def __init__(self, out_pkl_path: str, write_object: Any) -> None:
        with Path(expandvars(out_pkl_path)).open("wb") as pkl_file:
            pickle.dump(write_object, pkl_file)
