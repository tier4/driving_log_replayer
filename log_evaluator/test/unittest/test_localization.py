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
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from example_interfaces.msg import Float64
import pytest
from tier4_debug_msgs.msg import Float32Stamped
from tier4_debug_msgs.msg import Int32Stamped

from log_evaluator.localization import Availability
from log_evaluator.localization import Convergence
from log_evaluator.localization import ConvergenceCondition
from log_evaluator.localization import LocalizationScenario
from log_evaluator.localization import Reliability
from log_evaluator.localization import ReliabilityCondition
from log_evaluator.scenario import load_sample_scenario

TARGET_DIAG_NAME = "topic_state_monitor_ndt_scan_matcher_exe_time: localization_topic_status"


def test_scenario() -> None:
    scenario: LocalizationScenario = load_sample_scenario("localization", LocalizationScenario)
    assert scenario.VehicleId == "default"
    assert scenario.Evaluation.Conditions.Convergence.AllowableDistance == 0.2  # noqa


def test_availability_success() -> None:
    status = DiagnosticStatus(
        name=TARGET_DIAG_NAME,
        values=[KeyValue(key="status", value="OK")],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(status)
    assert evaluation_item.success is True
    assert evaluation_item.summary == "NDT Availability (Success): NDT available"
    assert frame_dict == {
        "Ego": {},
        "Availability": {
            "Result": {"Total": "Success", "Frame": "Success"},
            "Info": {},
        },
    }


def test_availability_fail() -> None:
    status = DiagnosticStatus(
        name=TARGET_DIAG_NAME,
        values=[
            KeyValue(key="not_availability_status", value="test"),
        ],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(status)
    assert evaluation_item.success is False
    assert evaluation_item.summary == "NDT Availability (Fail): NDT Availability Key Not Found"
    assert frame_dict == {
        "Ego": {},
        "Availability": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {},
        },
    }


def test_availability_fail_key_not_found() -> None:
    status = DiagnosticStatus(
        name=TARGET_DIAG_NAME,
        values=[KeyValue(key="status", value=Availability.ERROR_STATUS_LIST[0])],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(status)
    assert evaluation_item.success is False
    assert evaluation_item.summary == "NDT Availability (Fail): NDT not available"
    assert frame_dict == {
        "Ego": {},
        "Availability": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {},
        },
    }


@pytest.fixture()
def create_convergence() -> Convergence:
    condition = ConvergenceCondition(
        AllowableDistance=0.2,
        AllowableExeTimeMs=100.0,
        AllowableIterationNum=30,
        PassRate=95.0,
    )
    return Convergence(
        condition=condition,
        total=99,
        passed=94,
    )


def test_convergence_success(create_convergence: Callable) -> None:
    evaluation_item: Convergence = create_convergence
    frame_dict, pub_msg = evaluation_item.set_frame(
        0.1,
        0.2,
        {},
        Float32Stamped(data=50.0),
        Int32Stamped(data=10),
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Convergence (Success): 95 / 100 -> 95.00%"
    assert frame_dict == {
        "Ego": {"TransformStamped": {}},
        "Convergence": {
            "Result": {"Total": "Success", "Frame": "Success"},
            "Info": {
                "LateralDistance": 0.1,
                "HorizontalDistance": 0.2,
                "ExeTimeMs": 50.0,
                "IterationNum": 10,
            },
        },
    }
    assert pub_msg == Float64(data=0.1)


def test_convergence_fail(create_convergence: Callable) -> None:
    evaluation_item: Convergence = create_convergence
    frame_dict, pub_msg = evaluation_item.set_frame(
        0.3,
        0.2,
        {},
        Float32Stamped(data=50.0),
        Int32Stamped(data=10),
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Convergence (Fail): 94 / 100 -> 94.00%"
    assert frame_dict == {
        "Ego": {"TransformStamped": {}},
        "Convergence": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {
                "LateralDistance": 0.3,
                "HorizontalDistance": 0.2,
                "ExeTimeMs": 50.0,
                "IterationNum": 10,
            },
        },
    }
    assert pub_msg == Float64(data=0.3)


@pytest.fixture()
def create_reliability() -> Reliability:
    condition: ReliabilityCondition = ReliabilityCondition(
        Method="NVTL",
        AllowableLikelihood=2.3,
        NGCount=10,
    )
    return Reliability(
        condition=condition,
        total=9,
        ng_seq=9,
        received_data=[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9],
    )


def test_reliability_success(create_reliability: Callable) -> None:
    evaluation_item: Reliability = create_reliability
    nvtl = Float32Stamped(stamp=Time(sec=123, nanosec=456), data=2.3)
    # The function to create dict of map_to_baselink is checked in the evaluator test.
    map_to_baselink = {}
    tp = Float32Stamped(stamp=Time(sec=123, nanosec=123), data=2.3)
    frame_dict = evaluation_item.set_frame(nvtl, map_to_baselink, tp)
    assert evaluation_item.success is True
    assert (
        evaluation_item.summary
        == "Reliability (Success): NVTL Sequential NG Count: 0 (Total Test: 10, Average: 1.58000, StdDev: 0.34293)"
    )
    assert evaluation_item.received_data == [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.3]
    assert frame_dict == {
        "Ego": {"TransformStamped": {}},
        "Reliability": {
            "Result": {"Total": "Success", "Frame": "Success"},
            "Info": {
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
        },
    }


def test_reliability_fail(create_reliability: Callable) -> None:
    evaluation_item: Reliability = create_reliability
    nvtl = Float32Stamped(stamp=Time(sec=123, nanosec=456), data=2.0)
    # The function to create dict of map_to_baselink is checked in the evaluator test.
    map_to_baselink = {}
    tp = Float32Stamped(stamp=Time(sec=123, nanosec=123), data=2.3)
    frame_dict = evaluation_item.set_frame(nvtl, map_to_baselink, tp)
    assert evaluation_item.success is False
    assert (
        evaluation_item.summary
        == "Reliability (Fail): NVTL Sequential NG Count: 10 (Total Test: 10, Average: 1.55000, StdDev: 0.28723)"
    )
    assert evaluation_item.received_data == [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0]
    assert frame_dict == {
        "Ego": {"TransformStamped": {}},
        "Reliability": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {
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
        },
    }
