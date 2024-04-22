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

from driving_log_replayer.performance_diag import Blockage
from driving_log_replayer.performance_diag import BlockageCondition
from driving_log_replayer.performance_diag import PerformanceDiagScenario
from driving_log_replayer.performance_diag import Visibility
from driving_log_replayer.performance_diag import VisibilityCondition
from driving_log_replayer.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: PerformanceDiagScenario = load_sample_scenario(
        "performance_diag",
        PerformanceDiagScenario,
    )
    assert scenario.Evaluation.Conditions.LiDAR.Visibility.ScenarioType == "FP"
    assert scenario.Evaluation.Conditions.LiDAR.Blockage["front_lower"].BlockageType == "both"


def test_visibility_invalid() -> None:
    status = DiagnosticStatus(
        name="dual_return_filter: /sensing/lidar/front_lower: visibility_validation",
        level=DiagnosticStatus.OK,
    )
    evaluation_item = Visibility(
        condition=VisibilityCondition(ScenarioType=None, PassFrameCount=100),
    )
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
        condition=VisibilityCondition(ScenarioType="TP", PassFrameCount=100),
    )
    frame_dict, msg_visibility_value, msg_visibility_level = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert (
        evaluation_item.success is False
    )  # If there is no target status in the diag, SUCCESS is not updated. Default is false.
    assert evaluation_item.summary == "NotTested"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Warn"},
        "Info": {"Reason": "diagnostics does not contain visibility"},
    }
    assert msg_visibility_value is None
    assert msg_visibility_level is None


def test_visibility_tp_success() -> None:
    status = DiagnosticStatus(
        name="dual_return_filter: /sensing/lidar/front_lower: visibility_validation",
        level=DiagnosticStatus.ERROR,
        values=[KeyValue(key="value", value="-1.00")],
    )
    evaluation_item = Visibility(
        condition=VisibilityCondition(ScenarioType="TP", PassFrameCount=100),
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
        name="dual_return_filter: /sensing/lidar/front_lower: visibility_validation",
        level=DiagnosticStatus.OK,
        values=[KeyValue(key="value", value="1.00")],
    )
    evaluation_item = Visibility(
        condition=VisibilityCondition(ScenarioType="TP", PassFrameCount=100),
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
        name="dual_return_filter: /sensing/lidar/front_lower: visibility_validation",
        level=DiagnosticStatus.OK,
        values=[KeyValue(key="value", value="1.00")],
    )
    evaluation_item = Visibility(
        condition=VisibilityCondition(ScenarioType="FP", PassFrameCount=100),
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
        name="dual_return_filter: /sensing/lidar/front_lower: visibility_validation",
        level=DiagnosticStatus.ERROR,
        values=[KeyValue(key="value", value="-1.00")],
    )
    evaluation_item = Visibility(
        condition=VisibilityCondition(ScenarioType="FP", PassFrameCount=100),
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


def test_blockage_invalid() -> None:
    status = DiagnosticStatus(
        name="blockage_return_diag: /sensing/lidar/front_lower: blockage_validation",
        level=DiagnosticStatus.OK,
    )
    evaluation_item = Blockage(
        name="front_lower",
        condition=BlockageCondition(ScenarioType=None, BlockageType="both", PassFrameCount=100),
    )
    (
        frame_dict,
        msg_blockage_sky_ratio,
        msg_blockage_ground_ratio,
        msg_blockage_level,
    ) = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "Invalid"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Invalid"},
        "Info": {},
    }
    assert msg_blockage_sky_ratio is None
    assert msg_blockage_ground_ratio is None
    assert msg_blockage_level is None


def test_blockage_has_no_target_diag_not_target_lidar() -> None:
    status = DiagnosticStatus(
        name="blockage_return_diag: /sensing/lidar/rear_upper: blockage_validation",
    )
    evaluation_item = Blockage(
        name="front_lower",
        condition=BlockageCondition(ScenarioType="TP", BlockageType="both", PassFrameCount=100),
    )
    (
        frame_dict,
        msg_blockage_sky_ratio,
        msg_blockage_ground_ratio,
        msg_blockage_level,
    ) = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert (
        evaluation_item.success is False
    )  # If there is no target status in the diag, SUCCESS is not updated. Default is false.
    assert evaluation_item.summary == "NotTested"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Warn"},
        "Info": {"Reason": "diagnostics does not contain blockage"},
    }
    assert msg_blockage_sky_ratio is None
    assert msg_blockage_ground_ratio is None
    assert msg_blockage_level is None


def test_blockage_tp_success() -> None:
    status = DiagnosticStatus(
        name="blockage_return_diag: /sensing/lidar/front_lower: blockage_validation",
        level=DiagnosticStatus.ERROR,
        message="ERROR: LIDAR both blockage",
        values=[
            KeyValue(key="ground_blockage_ratio", value="0.194444"),
            KeyValue(key="ground_blockage_count", value="100"),
            KeyValue(key="sky_blockage_ratio", value="0.211706"),
            KeyValue(key="sky_blockage_count", value="100"),
        ],
    )
    evaluation_item = Blockage(
        name="front_lower",
        condition=BlockageCondition(ScenarioType="TP", BlockageType="both", PassFrameCount=100),
        success=False,
        passed=99,
        total=149,
    )
    (
        frame_dict,
        msg_blockage_sky_ratio,
        msg_blockage_ground_ratio,
        msg_blockage_level,
    ) = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "front_lower (Success): 100 / 150"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "Level": 2,
            "GroundBlockageRatio": 0.194444,
            "GroundBlockageCount": 100,
            "SkyBlockageRatio": 0.211706,
            "SkyBlockageCount": 100,
        },
    }
    assert msg_blockage_sky_ratio == Float64(data=0.211706)
    assert msg_blockage_ground_ratio == Float64(data=0.194444)
    assert msg_blockage_level == Byte(data=bytes([2]))


def test_blockage_tp_fail() -> None:
    status = DiagnosticStatus(
        name="blockage_return_diag: /sensing/lidar/front_lower: blockage_validation",
        level=DiagnosticStatus.ERROR,
        message="ERROR: LIDAR ground blockage",
        values=[
            KeyValue(key="ground_blockage_ratio", value="-1.000000"),
            KeyValue(key="ground_blockage_count", value="1"),
            KeyValue(key="sky_blockage_ratio", value="0.824167"),
            KeyValue(key="sky_blockage_count", value="100"),
        ],
    )
    evaluation_item = Blockage(
        name="front_lower",
        condition=BlockageCondition(ScenarioType="TP", BlockageType="both", PassFrameCount=100),
        success=False,
        passed=99,
        total=149,
    )
    (
        frame_dict,
        msg_blockage_sky_ratio,
        msg_blockage_ground_ratio,
        msg_blockage_level,
    ) = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "front_lower (Fail): 99 / 150"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "Level": 2,
            "GroundBlockageRatio": -1.0,
            "GroundBlockageCount": 1,
            "SkyBlockageRatio": 0.824167,
            "SkyBlockageCount": 100,
        },
    }
    assert msg_blockage_sky_ratio is None
    assert msg_blockage_ground_ratio is None
    assert msg_blockage_level is None


def test_blockage_fp_success() -> None:
    status = DiagnosticStatus(
        name="blockage_return_diag: /sensing/lidar/front_lower: blockage_validation",
        level=DiagnosticStatus.OK,
        message="OK: LIDAR sky blockage",
        values=[
            KeyValue(key="ground_blockage_ratio", value="0.000000"),
            KeyValue(key="ground_blockage_count", value="0"),
            KeyValue(key="sky_blockage_ratio", value="0.380967"),
            KeyValue(key="sky_blockage_count", value="50"),
        ],
    )
    evaluation_item = Blockage(
        name="front_lower",
        condition=BlockageCondition(ScenarioType="FP", BlockageType="both", PassFrameCount=100),
        success=True,
        passed=49,
        total=49,
    )
    (
        frame_dict,
        msg_blockage_sky_ratio,
        msg_blockage_ground_ratio,
        msg_blockage_level,
    ) = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is True
    assert evaluation_item.summary == "front_lower (Success): 50 / 50"
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "Level": 0,
            "GroundBlockageRatio": 0.0,
            "GroundBlockageCount": 0,
            "SkyBlockageRatio": 0.380967,
            "SkyBlockageCount": 50,
        },
    }
    assert msg_blockage_sky_ratio == Float64(data=0.380967)
    assert msg_blockage_ground_ratio == Float64(data=0.0)
    assert msg_blockage_level == Byte(data=bytes([0]))


def test_blockage_fp_fail() -> None:
    status = DiagnosticStatus(
        name="blockage_return_diag: /sensing/lidar/front_lower: blockage_validation",
        level=DiagnosticStatus.ERROR,
        message="ERROR: LIDAR both blockage",
        values=[
            KeyValue(key="ground_blockage_ratio", value="0.194444"),
            KeyValue(key="ground_blockage_count", value="50"),
            KeyValue(key="sky_blockage_ratio", value="0.211706"),
            KeyValue(key="sky_blockage_count", value="50"),
        ],
    )
    evaluation_item = Blockage(
        name="front_lower",
        condition=BlockageCondition(ScenarioType="FP", BlockageType="both", PassFrameCount=100),
        success=True,
        passed=49,
        total=49,
    )
    (
        frame_dict,
        msg_blockage_sky_ratio,
        msg_blockage_ground_ratio,
        msg_blockage_level,
    ) = evaluation_item.set_frame(
        DiagnosticArray(status=[status]),
    )
    assert evaluation_item.success is False
    assert evaluation_item.summary == "front_lower (Fail): 49 / 50"
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "Level": 2,
            "GroundBlockageRatio": 0.194444,
            "GroundBlockageCount": 50,
            "SkyBlockageRatio": 0.211706,
            "SkyBlockageCount": 50,
        },
    }
    assert msg_blockage_sky_ratio == Float64(data=0.211706)
    assert msg_blockage_ground_ratio == Float64(data=0.194444)
    assert msg_blockage_level == Byte(data=bytes([2]))
