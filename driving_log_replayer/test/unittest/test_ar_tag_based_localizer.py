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

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from driving_log_replayer.ar_tag_based_localizer import ArtagBasedLocalizerScenario
from driving_log_replayer.ar_tag_based_localizer import Availability
from driving_log_replayer.scenario import load_sample_scenario

TARGET_DIAG_NAME = "localization: ar_tag_based_localizer"


def test_scenario() -> None:
    scenario: ArtagBasedLocalizerScenario = load_sample_scenario(
        "ar_tag_based_localizer",
        ArtagBasedLocalizerScenario,
    )
    assert scenario.ScenarioName == "sample_ar_tag_based_localizer"


def test_availability_success() -> None:
    status = DiagnosticStatus(
        name=TARGET_DIAG_NAME,
        values=[KeyValue(key="Number of Detected AR Tags", value="1")],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(status)
    assert evaluation_item.success is True
    assert (
        evaluation_item.summary == "ArTagBasedLocalizer Availability (Success): Detected 1 AR tags"
    )
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
        values=[KeyValue(key="Number of Detected AR Tags", value="-1")],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(status)
    assert evaluation_item.success is False
    assert evaluation_item.summary == "ArTagBasedLocalizer Availability (Fail): Illegal value"
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
        values=[KeyValue(key="Convergence", value="AnyValue")],
    )
    evaluation_item = Availability()
    frame_dict = evaluation_item.set_frame(status)
    assert evaluation_item.success is False
    assert evaluation_item.summary == "ArTagBasedLocalizer Availability (Fail): Illegal value"
    assert frame_dict == {
        "Ego": {},
        "Availability": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {},
        },
    }
