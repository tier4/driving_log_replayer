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

from collections.abc import Callable
from os.path import expandvars

from pydantic import BaseModel
import pytest
from rclpy.clock import ClockType
from rclpy.clock import ROSClock
from rclpy.time import Time

from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter


class SampleResult(ResultBase):
    def __init__(self) -> None:
        super().__init__()

    def update(self) -> None:
        self._success = True
        self._summary = "Sample OK"

    def set_frame(self) -> None:
        self._frame = {"Test": "ok"}


class SampleConditions(BaseModel):
    PassRate: float


@pytest.fixture
def create_writer() -> ResultWriter:
    json_path = "$HOME/dlr_result.json"
    ros_clock = ROSClock()
    ros_time = Time(seconds=123, nanoseconds=456, clock_type=ClockType.ROS_TIME)
    ros_clock.set_ros_time_override(ros_time)
    ros_clock._set_ros_time_is_active(True)  # noqa
    condition = SampleConditions(PassRate=99.0)
    writer = ResultWriter(json_path, ros_clock, condition)
    yield writer
    # delete created jsonl file
    writer.delete_result_file()


def test_create_jsonl_path(create_writer: Callable) -> None:
    writer: ResultWriter = create_writer
    assert writer.result_path.as_posix() == expandvars("$HOME/dlr_result.jsonl")


def test_get_result(create_writer: Callable) -> None:
    writer: ResultWriter = create_writer
    sample_result = SampleResult()
    sample_result.set_frame()
    sample_result.update()
    record = writer.get_result(sample_result)
    writer.write_line(record)
    assert record["Result"]["Success"] is True
    assert record["Result"]["Summary"] == "Sample OK"
    assert record["Frame"] == {"Test": "ok"}
    # The system time stamp is not checked since it changes with each execution.
    # Since the logic is the same as that of ros time, it is sufficient to check ros time.
    assert record["Stamp"]["ROS"] == 123.000000456  # noqa
