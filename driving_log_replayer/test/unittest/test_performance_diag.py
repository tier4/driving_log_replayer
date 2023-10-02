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
from example_interfaces.msg import Byte
from example_interfaces.msg import Float64

from driving_log_replayer.performance_diag import Visibility


def test_visibility_invalid() -> None:
    status = DiagnosticStatus(
        name="/autoware/sensing/lidar/performance_monitoring/visibility/dual_return_filter:  sensing lidar left_upper: visibility_validation",
        level=DiagnosticStatus.OK,
    )
    evaluation_item = Visibility(condition={"ScenarioType": None, "PassFrameCount": 100})
    frame_dict, msg_visibility_value, msg_visibility_level = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Invalid"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Invalid"},
        "Info": {},
    }
    assert msg_visibility_value is None
    assert msg_visibility_level is None


def test_visibility_has_no_target_diag() -> None:
    status = DiagnosticStatus(name="not_visibility_diag_name")
    evaluation_item = Visibility(
        condition={"ScenarioType": "TP", "PassFrameCount": 100},
    )
    frame_dict, msg_visibility_value, msg_visibility_level = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "NotTested"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Warn"},
        "Info": {"Reason": "diagnostics does not contain visibility"},
    }
    assert msg_visibility_value is None
    assert msg_visibility_level is None


def test_visibility_tp_success() -> None:
    status = DiagnosticStatus(
        name="/autoware/sensing/lidar/performance_monitoring/visibility/dual_return_filter:  sensing lidar left_upper: visibility_validation",
        level=DiagnosticStatus.ERROR,
        values=[KeyValue(key="value", value="-1.00")],
    )
    evaluation_item = Visibility(
        condition={"ScenarioType": "TP", "PassFrameCount": 100},
        total=149,
        passed=99,
        success=False,
    )
    frame_dict, msg_visibility_value, msg_visibility_level = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Visibility (Success): 100 / 150"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "Level": 2,
            "Value": -1.0,
        },
    }
    assert msg_visibility_value is None
    assert msg_visibility_level is None


def test_visibility_tp_fail() -> None:
    status = DiagnosticStatus(
        name="/autoware/sensing/lidar/performance_monitoring/visibility/dual_return_filter:  sensing lidar left_upper: visibility_validation",
        level=DiagnosticStatus.OK,
        values=[KeyValue(key="value", value="1.00")],
    )
    evaluation_item = Visibility(
        condition={"ScenarioType": "TP", "PassFrameCount": 100},
        total=149,
        passed=99,
        success=False,
    )
    frame_dict, msg_visibility_value, msg_visibility_level = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Visibility (Fail): 99 / 150"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "Level": 0,
            "Value": 1.0,
        },
    }
    assert msg_visibility_value == Float64(data=1.0)
    assert msg_visibility_level == Byte(data=bytes([0]))


def test_visibility_fp_success() -> None:
    status = DiagnosticStatus(
        name="/autoware/sensing/lidar/performance_monitoring/visibility/dual_return_filter:  sensing lidar left_upper: visibility_validation",
        level=DiagnosticStatus.OK,
        values=[KeyValue(key="value", value="1.00")],
    )
    evaluation_item = Visibility(
        condition={"ScenarioType": "FP", "PassFrameCount": 100},
        total=49,
        passed=49,
        success=True,
    )
    frame_dict, msg_visibility_value, msg_visibility_level = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Visibility (Success): 50 / 50"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "Level": 0,
            "Value": 1.0,
        },
    }
    assert msg_visibility_value == Float64(data=1.0)
    assert msg_visibility_level == Byte(data=bytes([0]))


def test_visibility_fp_fail() -> None:
    status = DiagnosticStatus(
        name="/autoware/sensing/lidar/performance_monitoring/visibility/dual_return_filter:  sensing lidar left_upper: visibility_validation",
        level=DiagnosticStatus.ERROR,
        values=[KeyValue(key="value", value="-1.00")],
    )
    evaluation_item = Visibility(
        condition={"ScenarioType": "FP", "PassFrameCount": 100},
        total=49,
        passed=49,
        success=True,
    )
    frame_dict, msg_visibility_value, msg_visibility_level = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Visibility (Fail): 49 / 50"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "Level": 2,
            "Value": -1.0,
        },
    }
    assert msg_visibility_value is None
    assert msg_visibility_level is None
