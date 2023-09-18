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

from driving_log_replayer.localization import AvailabilityResult


def test_availability_success() -> None:
    status = DiagnosticStatus(
        name=AvailabilityResult.TARGET_DIAG_NAME,
        values=[KeyValue(key="status", value="OK")],
    )
    result = AvailabilityResult()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is True
    assert result.summary == "NDT Availability (Success): NDT available"
    assert frame == {
        "Availability": {
            "Result": "Success",
            "Info": [
                {},
            ],
        },
    }


def test_availability_fail() -> None:
    status = DiagnosticStatus(
        name=AvailabilityResult.TARGET_DIAG_NAME,
        values=[KeyValue(key="status", value=AvailabilityResult.ERROR_STATUS_LIST[0])],
    )
    result = AvailabilityResult()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is False
    assert result.summary == "NDT Availability (Fail): NDT not available"
    assert frame == {
        "Availability": {
            "Result": "Fail",
            "Info": [
                {},
            ],
        },
    }


def test_availability_has_no_target_diag() -> None:
    status = DiagnosticStatus(name="not_localization_diag_name")
    result = AvailabilityResult()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is True
    assert result.summary == "NotTested"
    assert frame == {
        "Availability": {
            "Result": "Fail",
            "Info": [
                {"Reason": "diagnostics does not contain localization_topic_status"},
            ],
        },
    }
