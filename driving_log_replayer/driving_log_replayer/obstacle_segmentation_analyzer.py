#!/usr/bin/env python3

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

from pathlib import Path
from typing import Dict
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
from driving_log_replayer_analyzer.calc import fail_3_times_in_a_row
from driving_log_replayer_analyzer.config import Config
from driving_log_replayer_analyzer.config import load_config
from driving_log_replayer_analyzer.jsonl_parser import JsonlParser
from driving_log_replayer_analyzer.plot.plot_base import PlotBase
import yaml


def default_config_path() -> Path:
    return Path(
        get_package_share_directory("driving_log_replayer_analyzer"),
        "config",
        "config.yaml",
    )


def update_config(config: Config, vehicle_model: str) -> Config:
    vehicle_info_param = Path(
        get_package_share_directory(vehicle_model + "_description"),
        "config",
        "vehicle_info.param.yaml",
    )
    with open(vehicle_info_param) as f:
        yaml_obj = yaml.safe_load(f)

    config.overhang_from_baselink = (
        yaml_obj["/**"]["ros__parameters"]["front_overhang"]
        + yaml_obj["/**"]["ros__parameters"]["wheel_base"]
    )
    return config


def get_graph_data(
    input_jsonl: Path, vehicle_model: str, output_dir: Path, config_yaml: Path
) -> Tuple[Dict, Dict]:
    output_dir.mkdir(exist_ok=True)

    # 設定ファイルのロード
    config = load_config(config_yaml)
    # ここにvehicle paramから更新するところ入れる
    config = update_config(config, vehicle_model)

    # Load result.jsonl
    parser = JsonlParser(input_jsonl, config)

    detection_dist_plot = PlotBase()
    detection_dist_plot.add_data(parser.get_bb_distance(), legend="1 Frame")

    min3_data = fail_3_times_in_a_row(parser.get_bb_distance())
    detection_dist_plot.add_data(min3_data, legend="3 Frames")

    pointcloud_numpoints_plot = PlotBase()
    for data in parser.get_pointcloud_points_per_uuid():
        pointcloud_numpoints_plot.add_data(data, legend=data[0][2])

    return detection_dist_plot._df.to_dict(), pointcloud_numpoints_plot._df.to_dict()
