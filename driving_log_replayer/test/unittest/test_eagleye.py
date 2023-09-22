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

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from driving_log_replayer.eagleye import AvailabilityResult


def test_availability_success() -> None:
    status = DiagnosticStatus(
        name=AvailabilityResult.TARGET_DIAG_NAME,
        level=DiagnosticStatus.OK,
        message="OK",
    )
    result = AvailabilityResult()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is True
    assert result.summary == "Eagleye Availability (Success): OK"
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Success",
            "Info": [],
        },
    }


def test_availability_fail() -> None:
    status = DiagnosticStatus(
        name=AvailabilityResult.TARGET_DIAG_NAME,
        level=DiagnosticStatus.WARN,
        message="not subscribed or deadlock of more than 10 seconds",
    )
    result = AvailabilityResult()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is False
    assert (
        result.summary
        == "Eagleye Availability (Fail): not subscribed or deadlock of more than 10 seconds"
    )
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Fail",
            "Info": [],
        },
    }


def test_availability_has_no_target_diag() -> None:
    status = DiagnosticStatus(name="not_eagleye_status")
    result = AvailabilityResult()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is False  # default value is False
    assert result.summary == "NotTested"
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Warn",
            "Info": [
                {"Reason": "diagnostics does not contain eagleye_enu_absolute_pos_interpolate"},
            ],
        },
    }
