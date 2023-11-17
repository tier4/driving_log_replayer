# Copyright (c) 2022 TIER IV.inc
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

from pathlib import Path
from typing import Any
from typing import Callable

from pydantic import BaseModel
from typing_extensions import Literal
import yaml


class Position(BaseModel):
    x: float
    y: float
    z: float


class Orientation(BaseModel):
    x: float
    y: float
    z: float
    w: float


class InitialPose(BaseModel):
    position: Position
    orientation: Orientation


class Scenario(BaseModel):
    ScenarioFormatVersion: Literal["2.2.0", "3.0.0"]
    ScenarioName: str
    ScenarioDescription: str
    SensorModel: str
    VehicleModel: str
    VehicleId: str | None = None
    LocalMapPath: str = ""
    Evaluation: Any


def load_scenario(scenario_path: Path, scenario_class: Callable) -> Any:
    with scenario_path.open() as scenario_file:
        return scenario_class(**yaml.safe_load(scenario_file))
