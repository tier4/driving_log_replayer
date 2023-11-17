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

# TODO: scenario loaderを作って直接sampleからdict作ってテストするのもあとで入れる


@pytest.fixture()
def create_scenario_dict() -> dict:
    return {
        "ScenarioFormatVersion": "2.2.0",
        "ScenarioName": "sample_localization",
        "ScenarioDescription": "sample_description",
        "SensorModel": "sample_sensor_kit",
        "VehicleModel": "sample_vehicle",
        "VehicleId": "default",
        "LocalMapPath": "$HOME/autoware_map/sample-map-planning",
        "Evaluation": {
            "UseCaseName": "localization",
            "UseCaseFormatVersion": "1.2.0",
            "Conditions": {
                "Convergence": {
                    "AllowableDistance": 0.2,
                    "AllowableExeTimeMs": 100.0,
                    "AllowableIterationNum": 30,
                    "PassRate": 95.0,
                },
                "Reliability": {
                    "Method": "NVTL",
                    "AllowableLikelihood": 2.3,
                    "NGCount": 10,
                },
            },
            # "InitialPose": {
            #     "position": {
            #         "x": 3836.5478515625,
            #         "y": 73729.96875,
            #         "z": 0.0,
            #     },
            #     "orientation": {
            #         "x": 0.0,
            #         "y": 0.0,
            #         "z": -0.9689404241590215,
            #         "w": 0.2472942668776119,
            #     },
            # },
        },
    }


@pytest.fixture()
def create_t4_scenario_dict() -> dict:
    return {
        "ScenarioFormatVersion": "3.0.0",
        "ScenarioName": "perception_use_bag_concat_data",
        "ScenarioDescription": "sensing_module_off_and_use_pointcloud_in_the_rosbag",
        "SensorModel": "sample_sensor_kit",
        "VehicleModel": "sample_vehicle",
        "Evaluation": {
            "UseCaseName": "perception",
            "UseCaseFormatVersion": "0.6.0",
            "Conditions": {
                "Convergence": {
                    "AllowableDistance": 0.2,
                    "AllowableExeTimeMs": 100.0,
                    "AllowableIterationNum": 30,
                    "PassRate": 95.0,
                },
                "Reliability": {
                    "Method": "NVTL",
                    "AllowableLikelihood": 2.3,
                    "NGCount": 10,
                },
            },
        },
    }


def test_scenario(create_scenario_dict: Callable) -> None:
    scenario_dict: dict = create_scenario_dict
    scenario = Scenario(**scenario_dict)
    print(scenario)


def test_t4_scenario(create_t4_scenario_dict: Callable) -> None:
    scenario_dict: dict = create_t4_scenario_dict
    scenario = Scenario(**scenario_dict)
    print(type(scenario.Evaluation))
    print(scenario)
