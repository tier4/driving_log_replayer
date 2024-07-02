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

from os.path import expandvars
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

from driving_log_replayer.launch_config import dlr_config
from driving_log_replayer.shutdown_once import ShutdownOnce


def get_launch_arguments() -> list:
    """
    Set and return launch argument.

    with_autoware
    scenario_path
    output_dir
    play_rate
    play_delay
    """
    launch_arguments = []

    def add_launch_arg(
        name: str,
        default_value: str | None = None,
        description: str = "",
    ) -> None:
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description),
        )

    add_launch_arg("scenario_path", description="scenario path")
    add_launch_arg("output_dir", description="output directory")
    add_launch_arg("dataset_index", default_value="0", description="index number of dataset")
    add_launch_arg("play_rate", default_value="1.0", description="ros2 bag play rate")
    add_launch_arg("play_delay", default_value="10.0", description="ros2 bag play delay")
    add_launch_arg(
        "with_autoware",
        default_value="true",
        description="Whether to launch autoware or not",
    )

    return launch_arguments


def ensure_arg_compatibility(context: LaunchContext) -> list:
    conf = context.launch_configurations
    scenario_path = Path(conf["scenario_path"])
    with scenario_path.open() as scenario_file:
        yaml_obj = yaml.safe_load(scenario_file)
    for k, v in yaml_obj["Evaluation"]["Datasets"][int(conf["dataset_index"])].items():
        dataset_path_str = expandvars(k)
        map_path_str = expandvars(v["LocalMapPath"])
        conf["vehicle_id"] = v["VehicleId"]
    map_path = Path(map_path_str)
    if not map_path.is_absolute():
        map_path = scenario_path.parent.joinpath(map_path)
    dataset_path = Path(dataset_path_str)
    if not dataset_path.is_absolute():
        dataset_path = scenario_path.parent.joinpath(dataset_path)
    conf["map_path"] = map_path.as_posix()
    conf["vehicle_model"] = yaml_obj["VehicleModel"]
    conf["sensor_model"] = yaml_obj["SensorModel"]
    conf["t4_dataset_path"] = dataset_path.as_posix()
    conf["input_bag"] = dataset_path.joinpath("input_bag").as_posix()
    output_dir = Path(conf["output_dir"])
    conf["result_json_path"] = output_dir.joinpath("result.json").as_posix()
    conf["result_bag_path"] = output_dir.joinpath("result_bag").as_posix()
    conf["result_archive_path"] = output_dir.joinpath("result_archive_path").as_posix()
    conf["use_case"] = yaml_obj["Evaluation"]["UseCaseName"]
    # annotationless
    conf["annotationless_threshold_file"] = ""
    conf["annotationless_pass_range"] = ""
    return [
        LogInfo(msg=f"{map_path=}, {dataset_path=}, use_case={conf['use_case']}"),
    ]


def launch_autoware(context: LaunchContext) -> list:
    conf = context.launch_configurations
    autoware_launch_file = Path(
        get_package_share_directory("autoware_launch"),
        "launch",
        "logging_simulator.launch.xml",
    )
    launch_args = {
        "map_path": conf["map_path"],
        "vehicle_model": conf["vehicle_model"],
        "sensor_model": conf["sensor_model"],
        "vehicle_id": conf["vehicle_id"],
        "launch_vehicle_interface": "true",
        "rviz": "false",
    }
    launch_args |= dlr_config[conf["use_case"]]["autoware"]
    return [
        GroupAction(
            [
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        autoware_launch_file.as_posix(),
                    ),
                    launch_arguments=launch_args.items(),
                    condition=IfCondition(conf["with_autoware"]),
                ),
            ],
            scoped=False,
            forwarding=True,
        ),
    ]


def launch_map_height_fitter(context: LaunchContext) -> list:
    fitter_launch_file = Path(
        get_package_share_directory("map_height_fitter"),
        "launch",
        "map_height_fitter.launch.xml",
    )
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                fitter_launch_file.as_posix(),
            ),
            condition=IfCondition(context.launch_configurations["localization"]),
        ),
    ]


def launch_evaluator_node(context: LaunchContext) -> list:
    conf = context.launch_configurations
    params = {
        "use_sim_time": True,
        "scenario_path": conf["scenario_path"],
        "t4_dataset_path": conf["t4_dataset_path"],
        "result_json_path": conf["result_json_path"],
        "result_archive_path": conf["result_archive_path"],
    }
    params |= dlr_config[conf["use_case"]]["node"]

    evaluator_name = conf["use_case"] + "_evaluator"

    return [
        Node(
            package="driving_log_replayer",
            namespace="/driving_log_replayer",
            executable=evaluator_name + "_node.py",
            output="screen",
            name=evaluator_name,
            parameters=[params],
            on_exit=ShutdownOnce(),
        ),
    ]


def launch_bag_player(
    context: LaunchContext,
) -> IncludeLaunchDescription:
    conf = context.launch_configurations
    play_cmd = [
        "ros2",
        "bag",
        "play",
        conf["input_bag"],
        "--delay",
        conf["play_delay"],
        "--rate",
        conf["play_rate"],
        "--clock",
        "200",
    ]
    remap_list = ["--remap"]
    if conf.get("sensing", "true") == "true":
        remap_list.append(
            "/sensing/lidar/concatenated/pointcloud:=/dlr/unused/concatenated/pointcloud",
        )
    if conf.get("localization", "true") == "true":
        remap_list.append(
            "/tf:=/dlr/unused/tf",
        )
    if len(remap_list) != 1:
        play_cmd.extend(remap_list)
    bag_player = ExecuteProcess(cmd=play_cmd, output="screen")
    return [bag_player]


def launch_bag_recorder(context: LaunchContext) -> list:
    conf = context.launch_configurations
    record_cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        conf["result_bag_path"],
        "--qos-profile-overrides-path",
        Path(
            get_package_share_directory("driving_log_replayer"),
            "config",
            conf["use_case"],
            "qos.yaml",
        ).as_posix(),
        "-e",
        dlr_config[conf["use_case"]]["record"],
        "--use-sim-time",
    ]
    return [ExecuteProcess(cmd=record_cmd)]


def launch_rviz(context: LaunchContext) -> list:
    conf = context.launch_configurations
    rviz_config_dir = Path(
        get_package_share_directory("driving_log_replayer"),
        "config",
        "dlr.rviz",
    )
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_dir.as_posix()],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(conf.get("rviz", "true")),
        ),
    ]


def launch_topic_state_monitor(context: LaunchContext) -> list:
    conf = context.launch_configurations
    # component_state_monitor launch
    component_state_monitor_launch_file = Path(
        get_package_share_directory("component_state_monitor"),
        "launch",
        "component_state_monitor.launch.py",
    )
    topic_monitor_config_path = Path(
        get_package_share_directory("driving_log_replayer"),
        "config",
        conf["use_case"],
        "topic_state_monitor.yaml",
    )
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                component_state_monitor_launch_file.as_posix(),
            ),
            launch_arguments={
                "file": topic_monitor_config_path.as_posix(),
                "mode": "logging_simulation",
            }.items(),
            condition=IfCondition(str(conf["use_case"] == "localization")),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=ensure_arg_compatibility),
            OpaqueFunction(function=launch_rviz),
            OpaqueFunction(function=launch_autoware),
            OpaqueFunction(function=launch_map_height_fitter),
            OpaqueFunction(function=launch_evaluator_node),
            OpaqueFunction(function=launch_bag_player),
            OpaqueFunction(function=launch_bag_recorder),
            OpaqueFunction(function=launch_topic_state_monitor),
        ],
    )
