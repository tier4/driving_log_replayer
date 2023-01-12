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

from driving_log_replayer_analyzer.config.obstacle_segmentation import load_config
from driving_log_replayer_analyzer.data.obstacle_segmentation import JsonlParser
from driving_log_replayer_analyzer.data.obstacle_segmentation import fail_3_times_in_a_row
from driving_log_replayer_analyzer.plot.bird_view_plot import BirdViewPlot
from driving_log_replayer_analyzer.plot.line_plot import LinePlot
from driving_log_replayer_analyzer.plot.scatter_plot import ScatterPlot


def visualize(input_jsonl: Path, output_dir: Path, config_yaml: Path):
    output_dir.mkdir(exist_ok=True)

    # 設定ファイルのロード
    config = load_config(config_yaml)

    # Load result.jsonl
    parser = JsonlParser(input_jsonl, config)

    # Create summary
    parser.summary.save(output_dir / "0_summary")

    # Plot
    bird_view_plot = BirdViewPlot()
    bird_view_plot.add_data(parser.get_bb_position())
    bird_view_plot.add_data(parser.get_pointcloud_position())
    bird_view_plot.plot(
        title="車両先端～Annotation BoundingBox(BB)の中心点(x,y)とDetectionのPass/Fail",
        xlabel="y[m]",
        ylabel="x[m]",
    )
    bird_view_plot.set_scale(config.bird_view_scale, config.bird_view_origin)
    bird_view_plot.set_tick_span(x=5.0, y=5.0)
    bird_view_plot.save_plot(
        output_dir / "1_ego_to_bb_detection_result",
    )

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

    topic_rate_plot = LinePlot()
    topic_rate_plot.add_data(parser.get_topic_rate(), legend="Rate")
    topic_rate_plot.plot(title="Frame rate (0.5倍速)", xlabel="Frame", ylabel="周期[Hz]")
    topic_rate_plot.save_plot(
        output_dir / "3_frame_rate",
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

    pointcloud_diff_plot = LinePlot()
    for data in parser.get_annotation_and_pointcloud_distance():
        pointcloud_diff_plot.add_data(data, legend=data[0][2])
    pointcloud_diff_plot.plot(
        title="車両先端～Annotation BBの最近傍点と検知点群の最近傍点との距離差",
        xlabel="車両先端~Annotation BBの最近傍点の距離[m]",
        ylabel="Pointcloudの最近傍点とAnnotation BBの距離差[m]",
    )
    pointcloud_diff_plot.save_plot(
        output_dir / "5_diff_bb_and_points",
    )

    pointcloud_non_detection_plot = ScatterPlot()
    pointcloud_non_detection_plot.add_data_with_hover(
        parser.get_non_detection_frame_points(config.fp_distance), legend="points"
    )
    pointcloud_non_detection_plot.plot_with_hover(
        title="False Positive",
        xlabel="Frame number",
        ylabel="Non detection領域の検知点数",
    )
    pointcloud_non_detection_plot.save_plot(
        output_dir / "6_false_positive_points_per_frame",
    )

    non_detection_tf_plot = BirdViewPlot()
    non_detection_tf_plot.add_data_with_hover(parser.get_non_detection_position(config.fp_distance))
    non_detection_tf_plot.plot_with_hover(
        title="非検知領域での検知点群数",
        xlabel="y[m]",
        ylabel="x[m]",
    )
    non_detection_tf_plot.set_scale(config.bird_view_scale, origin=False)
    non_detection_tf_plot.set_tick_span(5.0)
    non_detection_tf_plot.save_plot(
        output_dir / "7_false_positive_points_per_position",
    )

    # Output data to csv
    parser.export_to_csv(output_dir / "result.csv")
