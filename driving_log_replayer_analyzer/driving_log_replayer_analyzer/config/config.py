# Copyright (c) 2022 TierIV.inc
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

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from ..data.non_detection import FpDistance


@dataclass
class Config:
    overhang_from_baselink: float
    fp_distance: FpDistance
    bird_view_scale: Any
    bird_view_origin: bool


def load_config(file: Path) -> Config:
    with open(file) as f:
        yaml_obj = yaml.safe_load(f)

    return Config(
        yaml_obj["vehicle"]["front_overhang"] + yaml_obj["vehicle"]["wheel_base"],
        FpDistance(
            yaml_obj["fp_threshold"]["very_near"],
            yaml_obj["fp_threshold"]["near"],
            yaml_obj["fp_threshold"]["medium"],
            yaml_obj["fp_threshold"]["far"],
        ),
        yaml_obj["bird_view"]["scale"],
        yaml_obj["bird_view"]["origin"],
    )
