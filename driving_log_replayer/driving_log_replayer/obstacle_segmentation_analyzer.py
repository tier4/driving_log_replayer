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

import argparse
import os
from pathlib import Path
from typing import Dict
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
from driving_log_replayer_analyzer.calc import fail_3_times_in_a_row
from driving_log_replayer_analyzer.config import Config
from driving_log_replayer_analyzer.config import load_config
from driving_log_replayer_analyzer.jsonl_parser import JsonlParser
from driving_log_replayer_analyzer.plot.scatter_plot import ScatterPlot
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

    detection_dist_plot = ScatterPlot()
    detection_dist_plot.add_data(parser.get_bb_distance(), legend="1 Frame")

    min3_data = fail_3_times_in_a_row(parser.get_bb_distance())
    detection_dist_plot.add_data(min3_data, legend="3 Frames")

    pointcloud_numpoints_plot = ScatterPlot()
    for data in parser.get_pointcloud_points_per_uuid():
        pointcloud_numpoints_plot.add_data(data, legend=data[0][2])

    return detection_dist_plot._df.to_dict(), pointcloud_numpoints_plot._df.to_dict()


def visualize(input_jsonl: Path, vehicle_model: str, output_dir: Path, config_yaml: Path):
    output_dir.mkdir(exist_ok=True)

    # 設定ファイルのロード
    config = load_config(config_yaml)
    # ここにvehicle paramから更新するところ入れる
    config = update_config(config, vehicle_model)

    # Load result.jsonl
    parser = JsonlParser(input_jsonl, config)

    detection_dist_plot = ScatterPlot()
    detection_dist_plot.add_data(parser.get_bb_distance(), legend="1 Frame")

    min3_data = fail_3_times_in_a_row(parser.get_bb_distance())
    detection_dist_plot.add_data(min3_data, legend="3 Frames")
    detection_dist_plot.plot(
        title=f"最大検知距離 (1frame={parser.summary.visible_range_one_frame:.3f}m, 3frames={parser.summary.visible_range_three_frame:.3f}m)",
        xlabel="距離[m]",
        ylabel="Pass/Fail",
    )
    detection_dist_plot.use_boolean_tick()
    detection_dist_plot.set_tick_span(x=5.0)
    detection_dist_plot.save_plot(
        output_dir / "2_detection_distance",
    )

    pointcloud_numpoints_plot = ScatterPlot()
    for data in parser.get_pointcloud_points_per_uuid():
        pointcloud_numpoints_plot.add_data(data, legend=data[0][2])
    pointcloud_numpoints_plot.plot(
        title="車両先端～Annotation BB内のPointCloud最近傍点の距離ごとの検知点群数",
        xlabel="距離[m]",
        ylabel="検知点群数",
    )
    pointcloud_numpoints_plot.save_plot(
        output_dir / "4_ego_to_bb_points",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i", "--input_file", required=True, help="Input file (result.jsonl)", type=str
    )
    parser.add_argument("-v", "--vehicle", required=True, help="Vehicle Model Name", type=str)
    parser.add_argument(
        "-o",
        "--output_dir",
        help="Output directory",
        required=False,
        type=str,
    )
    parser.add_argument(
        "-c",
        "--config",
        help="Config file",
        default=default_config_path(),
        type=str,
    )
    args = parser.parse_args()
    p_input_file = Path(os.path.expandvars(args.input_file))
    if args.output_dir:
        p_output_dir = Path(os.path.expandvars(args.output_dir))
    else:
        p_output_dir = p_input_file.parent
    p_config = Path(os.path.expandvars(args.config))
    visualize(p_input_file, args.vehicle, p_output_dir, p_config)


if __name__ == "__main__":
    main()
