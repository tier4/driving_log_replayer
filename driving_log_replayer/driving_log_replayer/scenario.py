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

from pydantic import BaseModel
from pydantic import Field
from pydantic import validator


class ScenarioFormatVersionError(Exception):
    pass


class UseCaseFormatVersionError(Exception):
    pass


class PositionConfig(BaseModel):
    x: float
    y: float
    z: float


class OrientationConfig(BaseModel):
    x: float
    y: float
    z: float
    w: float


class InitialPoseConfig(BaseModel):
    position: PositionConfig
    orientation: OrientationConfig


class Scenario(BaseModel):
    ScenarioFormatVersion: str = Field(pattern="3.0.0")
    ScenarioName: str
    ScenarioDescription: str
    SensorModel: str
    VehicleModel: str
    # Evaluation: BaseModel


class ScenarioWithoutT4Dataset(Scenario):
    ScenarioFormatVersion: str = Field(pattern="2.2.0")
    VehicleId: str
    LocalMapPath: str = Field(default="")
