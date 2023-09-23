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

from driving_log_replayer.yabloc import Availability


def test_availability_success() -> None:
    status = DiagnosticStatus(
        name=Availability.TARGET_DIAG_NAME,
        values=[KeyValue(key="Availability", value="OK")],
    )
    result = Availability()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is True
    assert result.summary == "Yabloc Availability (Success): OK"
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Success",
            "Info": [],
        },
    }


def test_availability_fail() -> None:
    status = DiagnosticStatus(
        name=Availability.TARGET_DIAG_NAME,
        values=[KeyValue(key="Availability", value="NG")],
    )
    result = Availability()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is False
    assert result.summary == "Yabloc Availability (Fail): NG"
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Fail",
            "Info": [],
        },
    }


def test_availability_fail_key_not_found() -> None:
    status = DiagnosticStatus(
        name=Availability.TARGET_DIAG_NAME,
        values=[KeyValue(key="Convergence", value="AnyValue")],
    )
    result = Availability()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is False
    assert result.summary == "Yabloc Availability (Fail): Availability Key Not Found"
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Fail",
            "Info": [],
        },
    }


def test_availability_has_no_target_diag() -> None:
    status = DiagnosticStatus(name="not_yabloc_status")
    result = Availability()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is False  # default value is False
    assert result.summary == "NotTested"
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Warn",
            "Info": [
                {"Reason": "diagnostics does not contain yabloc_status"},
            ],
        },
    }
