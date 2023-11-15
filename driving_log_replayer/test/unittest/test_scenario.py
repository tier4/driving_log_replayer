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

from typing import Callable

import pytest

from driving_log_replayer.scenario import Scenario
from driving_log_replayer.scenario import ScenarioWithoutT4Dataset

# TODO: scenario loaderを作って直接sampleからdict作ってテストするのもあとで入れる


@pytest.fixture()
def create_scenario_dict() -> dict:
    return {
        "ScenarioFormatVersion": "3.0.0",
        "ScenarioName": "perception_use_bag_concat_data",
        "ScenarioDescription": "sensing_module_off_and_use_pointcloud_in_the_rosbag",
        "SensorModel": "sample_sensor_kit",
        "VehicleModel": "sample_vehicle",
    }


@pytest.fixture()
def create_scenario_without_t4_dataset_dict() -> dict:
    return {
        "ScenarioFormatVersion": "3.0.0",
        "ScenarioName": "perception_use_bag_concat_data",
        "ScenarioDescription": "sensing_module_off_and_use_pointcloud_in_the_rosbag",
        "SensorModel": "sample_sensor_kit",
        "VehicleModel": "sample_vehicle",
        "VehicleId": "ps1",
    }


def test_scenario(create_scenario_dict: Callable) -> None:
    scenario_dict: dict = create_scenario_dict
    scenario = Scenario(**scenario_dict)
    assert scenario.ScenarioDescription == "sensing_module_off_and_use_pointcloud_in_the_rosbag"


def test_scenario_without_t4_dataset(create_scenario_without_t4_dataset_dict: Callable) -> None:
    scenario_dict: dict = create_scenario_without_t4_dataset_dict
    scenario = Scenario(**scenario_dict)
    assert scenario.ScenarioDescription == "sensing_module_off_and_use_pointcloud_in_the_rosbag"
