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

from driving_log_replayer.scenario import load_sample_scenario
from driving_log_replayer.yabloc import Availability
from driving_log_replayer.yabloc import YablocScenario


def test_scenario() -> None:
    scenario: YablocScenario = load_sample_scenario(
        "yabloc",
        YablocScenario,
    )
    assert scenario.ScenarioName == "sample_yabloc"


def test_availability_success() -> None:
    status = DiagnosticStatus(
        name=Availability.TARGET_DIAG_NAME,
        values=[KeyValue(key="Availability", value="OK")],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(DiagnosticArray(status=[status]))
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Yabloc Availability (Success): OK"
    assert frame_dict == {
        "Ego": {},
        "Availability": {
            "Result": {"Total": "Success", "Frame": "Success"},
            "Info": {},
        },
    }


def test_availability_fail() -> None:
    status = DiagnosticStatus(
        name=Availability.TARGET_DIAG_NAME,
        values=[KeyValue(key="Availability", value="NG")],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(DiagnosticArray(status=[status]))
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Yabloc Availability (Fail): NG"
    assert frame_dict == {
        "Ego": {},
        "Availability": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {},
        },
    }


def test_availability_fail_key_not_found() -> None:
    status = DiagnosticStatus(
        name=Availability.TARGET_DIAG_NAME,
        values=[KeyValue(key="Convergence", value="AnyValue")],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(DiagnosticArray(status=[status]))
    assert evaluation_item.success is False
    assert evaluation_item.summary == "Yabloc Availability (Fail): Availability Key Not Found"
    assert frame_dict == {
        "Ego": {},
        "Availability": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {},
        },
    }


def test_availability_has_no_target_diag() -> None:
    status = DiagnosticStatus(name="not_yabloc_status")
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(DiagnosticArray(status=[status]))
    assert evaluation_item.success is False  # default value is False
    assert evaluation_item.summary == "NotTested"
    assert frame_dict == {
        "Ego": {},
        "Availability": {
            "Result": {"Total": "Fail", "Frame": "Warn"},
            "Info": {"Reason": "diagnostics does not contain yabloc_status"},
        },
    }
