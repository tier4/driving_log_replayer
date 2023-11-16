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
from typing_extensions import Literal


class ScenarioFormatVersionError(Exception):
    pass


class UseCaseFormatVersionError(Exception):
    pass


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


class EvaluationModel(BaseModel):
    UseCaseName: str
    UseCaseFormatVersion: str
    Conditions: dict
    InitialPose: InitialPose | None


class Localization(EvaluationModel):
    UseCaseName: Literal["localization"]
    UseCaseFormatVersion: Literal["1.2.0"]
    Conditions: dict


class Perception(EvaluationModel):
    UseCaseName: Literal["perception"]
    UseCaseFormatVersion: Literal["0.6.0"]
    Conditions: dict


Evaluation = Perception | Localization


class Scenario(BaseModel):
    ScenarioFormatVersion: Literal["2.2.0", "3.0.0"]
    ScenarioName: str
    ScenarioDescription: str
    SensorModel: str
    VehicleModel: str
    VehicleId: str | None = Field(default=None)
    LocalMapPath: str = Field(default="")
    Evaluation: Evaluation
