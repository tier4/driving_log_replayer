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

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from example_interfaces.msg import Float64
import pytest
from tier4_debug_msgs.msg import Float32Stamped
from tier4_debug_msgs.msg import Int32Stamped

from driving_log_replayer.localization import Availability
from driving_log_replayer.localization import Convergence
from driving_log_replayer.localization import get_reliability_method
from driving_log_replayer.localization import Reliability


def test_availability_success() -> None:
    status = DiagnosticStatus(
        name=Availability.TARGET_DIAG_NAME,
        values=[KeyValue(key="status", value="OK")],
    )
    result = Availability()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is True
    assert result.summary == "NDT Availability (Success): NDT available"
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
        values=[
            KeyValue(key="not_availability_status", value="test"),
        ],
    )
    result = Availability()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is False
    assert result.summary == "NDT Availability (Fail): NDT Availability Key Not Found"
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
        values=[KeyValue(key="status", value=Availability.ERROR_STATUS_LIST[0])],
    )
    result = Availability()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is False
    assert result.summary == "NDT Availability (Fail): NDT not available"
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Fail",
            "Info": [],
        },
    }


def test_availability_has_no_target_diag() -> None:
    status = DiagnosticStatus(name="not_localization_diag_name")
    result = Availability()
    frame = result.set_frame(DiagnosticArray(status=[status]))
    assert result.success is True
    assert result.summary == "NotTested"
    assert frame == {
        "Ego": {},
        "Availability": {
            "Result": "Warn",
            "Info": [
                {"Reason": "diagnostics does not contain localization_topic_status"},
            ],
        },
    }


@pytest.fixture()
def create_convergence() -> Convergence:
    return Convergence(
        condition={
            "AllowableDistance": 0.2,
            "AllowableExeTimeMs": 100.0,
            "AllowableIterationNum": 30,
            "PassRate": 95.0,
        },
        total=99,
        passed=94,
    )


def test_convergence_success(create_convergence: Callable) -> None:
    result: Convergence = create_convergence
    frame, pub_msg = result.set_frame(
        0.1,
        0.2,
        {},
        Float32Stamped(data=50.0),
        Int32Stamped(data=10),
    )
    assert result.success is True
    assert result.summary == "Convergence (Success): 95 / 100 -> 95.00%"
    assert frame == {
        "Ego": {"TransformStamped": {}},
        "Convergence": {
            "Result": "Success",
            "Info": [
                {
                    "LateralDistance": 0.1,
                    "HorizontalDistance": 0.2,
                    "ExeTimeMs": 50.0,
                    "IterationNum": 10,
                },
            ],
        },
    }
    assert pub_msg == Float64(data=0.1)


def test_convergence_fail(create_convergence: Callable) -> None:
    result: Convergence = create_convergence
    frame, pub_msg = result.set_frame(
        0.3,
        0.2,
        {},
        Float32Stamped(data=50.0),
        Int32Stamped(data=10),
    )
    assert result.success is False
    assert result.summary == "Convergence (Fail): 94 / 100 -> 94.00%"
    assert frame == {
        "Ego": {"TransformStamped": {}},
        "Convergence": {
            "Result": "Fail",
            "Info": [
                {
                    "LateralDistance": 0.3,
                    "HorizontalDistance": 0.2,
                    "ExeTimeMs": 50.0,
                    "IterationNum": 10,
                },
            ],
        },
    }
    assert pub_msg == Float64(data=0.3)


@pytest.fixture()
def create_reliability() -> Reliability:
    return Reliability(
        condition={"Method": "NVTL", "AllowableLikelihood": 2.3, "NGCount": 10},
        total=9,
        ng_seq=9,
        received_data=[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9],
    )


def test_reliability_success(create_reliability: Callable) -> None:
    result: Reliability = create_reliability
    nvtl = Float32Stamped(stamp=Time(sec=123, nanosec=456), data=2.3)
    # The function to create dict of map_to_baselink is checked in the evaluator test.
    map_to_baselink = {}
    tp = Float32Stamped(stamp=Time(sec=123, nanosec=123), data=2.3)
    frame = result.set_frame(nvtl, map_to_baselink, tp)
    assert result.success is True
    assert (
        result.summary
        == "Reliability (Success): NVTL Sequential NG Count: 0 (Total Test: 10, Average: 1.58000, StdDev: 0.34293)"
    )
    assert result.received_data == [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.3]
    assert frame == {
        "Ego": {"TransformStamped": {}},
        "Reliability": {
            "Result": "Success",
            "Info": [
                {
                    "Value": {
                        "stamp": {
                            "sec": 123,
                            "nanosec": 456,
                        },
                        "data": 2.3,
                    },
                    "Reference": {
                        "stamp": {
                            "sec": 123,
                            "nanosec": 123,
                        },
                        "data": 2.3,
                    },
                },
            ],
        },
    }


def test_reliability_fail(create_reliability: Callable) -> None:
    result: Reliability = create_reliability
    nvtl = Float32Stamped(stamp=Time(sec=123, nanosec=456), data=2.0)
    # The function to create dict of map_to_baselink is checked in the evaluator test.
    map_to_baselink = {}
    tp = Float32Stamped(stamp=Time(sec=123, nanosec=123), data=2.3)
    frame = result.set_frame(nvtl, map_to_baselink, tp)
    assert result.success is False
    assert (
        result.summary
        == "Reliability (Fail): NVTL Sequential NG Count: 10 (Total Test: 10, Average: 1.55000, StdDev: 0.28723)"
    )
    assert result.received_data == [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0]
    assert frame == {
        "Ego": {"TransformStamped": {}},
        "Reliability": {
            "Result": "Fail",
            "Info": [
                {
                    "Value": {
                        "stamp": {
                            "sec": 123,
                            "nanosec": 456,
                        },
                        "data": 2.0,
                    },
                    "Reference": {
                        "stamp": {
                            "sec": 123,
                            "nanosec": 123,
                        },
                        "data": 2.3,
                    },
                },
            ],
        },
    }


def test_get_reliability_method_success() -> None:
    reliability_method, error_msg = get_reliability_method("NVTL")
    assert reliability_method == "NVTL"
    assert error_msg == ""


def test_get_reliability_method_fail() -> None:
    reliability_method, error_msg = get_reliability_method("invalid_method")
    assert reliability_method is None
    assert error_msg == "invalid_method is not valid reliability method"


def test_get_reliability_method_scenario_format_error() -> None:
    reliability_method, error_msg = get_reliability_method(None)
    assert reliability_method is None
    assert error_msg == "Scenario format error"
