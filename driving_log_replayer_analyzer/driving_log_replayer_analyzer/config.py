from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .data.non_detection import FpDistance


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
