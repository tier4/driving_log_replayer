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
import launch
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from driving_log_replayer.shutdown_once import ShutdownOnce

"""
ros2 launch driving_log_replayer localization.launch.py
map_path:=/home/hyt/map/oss シナリオに書いてある、webautoだと、パスを変えるので、map_path指定されたら強制上書き
vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit　シナリオに書いてある
vehicle_id:=default scenario_path:=/home/hyt/dlr_data/aw/localization/sample/scenario.yaml
result_json_path:=/home/hyt/out/aw/localization/2024-0605-181534/sample/result.json outputを指定すれば自動で作れる
input_bag:=/home/hyt/dlr_data/aw/localization/sample/input_bag datasetを書いてあれば自動で
result_bag_path:=/home/hyt/out/aw/localization/2024-0605-181534/sample/result_bag
"""


def get_launch_arguments() -> list:
    """
    Set and return launch argument.

    with_autoware
    scenario_path
    output_dir
    play_rate
    play_delay
    override_record_topics
    override_topics_regex

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

    add_launch_arg(
        "with_autoware",
        default_value="true",
        description="Whether to launch autoware or not",
    )
    add_launch_arg("scenario_path", description="scenario path")
    add_launch_arg("output_dir", description="output directory")
    add_launch_arg("play_rate", default_value="1.0", description="ros2 bag play rate")
    add_launch_arg("play_delay", default_value="10.0", description="ros2 bag play delay")
    add_launch_arg(
        "override_record_topics",
        default_value="false",
        description="flag of override record topics",
    )
    add_launch_arg(
        "override_topics_regex",
        default_value="",
        description="use allowlist. Ex: override_topics_regex:=\^/clock\$\|\^/tf\$\|/sensing/lidar/concatenated/pointcloud",  # noqa
    )

    return launch_arguments


def ensure_arg_compatibility(context: LaunchContext) -> list:
    conf = context.launch_configurations
    scenario_path = Path(conf["scenario_path"])
    with scenario_path.open() as scenario_file:
        yaml_obj = yaml.safe_load(scenario_file)
    for k, v in yaml_obj["Evaluation"]["Datasets"][0].items():
        dataset_path_str = expandvars(k)
        map_path_str = expandvars(v["LocalMapPath"])
    map_path = Path(map_path_str)
    if not map_path.is_absolute():
        map_path = scenario_path.parent.joinpath(map_path)
    dataset_path = Path(dataset_path_str)
    if not dataset_path.is_absolute():
        dataset_path = scenario_path.parent.joinpath(dataset_path)
    conf["map_path"] = map_path.as_posix()
    conf["dataset_path"] = dataset_path.as_posix()
    # add configurations
    conf["input_bag"] = dataset_path.joinpath("input_bag").as_posix()
    conf["use_case"] = yaml_obj["Evaluation"]["UseCaseName"]
    return [
        LogInfo(msg=f"{map_path=}, {dataset_path=}"),
    ]


def launch_autoware(context: LaunchContext) -> list:
    autoware_launch_file = Path(
        get_package_share_directory("autoware_launch"),
        "launch",
        "logging_simulator.launch.xml",
    )
    launch_args = {
        "map_path": conf.get(
            "map_path",
            yaml_obj["Evaluation"]["Datasets"][0]["sample_dataset"]["LocalMapPath"],
        ),
        "vehicle_model": conf.get("vehicle_model", yaml_obj["VehicleModel"]),
        "sensor_model": conf.get("sensor_model", yaml_obj["SensorModel"]),
        "vehicle_id": conf.get("vehicle_id", yaml_obj["VehicleId"]),
        "launch_vehicle_interface": "true",
        # "rviz": "false",
    }
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


def launch_evaluator_node(context: LaunchContext, addition_parameter: dict | None) -> list:
    conf = context.launch_configurations
    scenario_path = Path(conf["scenario_path"])
    with scenario_path.open() as scenario_file:
        yaml_obj: dict = yaml.safe_load(scenario_file)

    params = {
        "use_sim_time": True,
        "scenario_path": conf["scenario_path"],
        "result_json_path": conf.get("result_json_path", Path(conf["output_dir"], "result.jsonl")),
        "t4_dataset_path": conf.get(
            "t4_dataset_path",
            yaml_obj["Evaluation"]["Datasets"]["0"]["sample_dataset"],
        ),
        "result_archive_path": conf.get(
            "result_archive_path",
            Path(conf["output_dir"], "result_archive"),
        ),
    }
    if addition_parameter is not None and isinstance(addition_parameter, dict):
        params.update(addition_parameter)

    node_name = conf["dlr_use_case"] + "_evaluator_node.py"

    return [
        Node(
            package="driving_log_replayer",
            namespace="/driving_log_replayer",
            executable=node_name,
            output="screen",
            name=conf["dlr_use_case"] + "_evaluator",
            parameters=[params],
            on_exit=ShutdownOnce(),
        ),
    ]


def launch_bag_player(
    context: LaunchContext,
    additional_argument: list | None = None,
) -> IncludeLaunchDescription:
    conf = context.launch_configurations
    input_bag = conf.get("input_bag", Path(conf["dataset_path"], "input_bag").as_posix())
    play_cmd = (
        [
            "ros2",
            "bag",
            "play",
            input_bag,
            "--delay",
            conf["play_delay"],
            "--rate",
            conf["play_rate"],
            "--clock",
            "200",
        ],
    )
    if additional_argument is not None and isinstance(additional_argument, list):
        play_cmd.extend(additional_argument)
    bag_player = ExecuteProcess(play_cmd, output="screen")
    return [bag_player]


def launch_recorder(context: LaunchContext) -> list:
    conf = context.launch_configurations
    record_cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        conf("result_bag_path"),
        "--qos-profile-overrides-path",
        Path(
            get_package_share_directory("driving_log_replayer"),
            "config",
            record_config_name,
        ).as_posix(),
        "-e",
        allowlist,
        "--use-sim-time",
    ]
    return [ExecuteProcess(cmd=record_cmd)]


def get_regex_recorder(record_config_name: str, allowlist: str) -> ExecuteProcess:
    record_cmd = get_regex_record_cmd(record_config_name, allowlist)
    return ExecuteProcess(cmd=record_cmd)


def get_regex_recorders(
    record_config_name: str,
    allowlist: str,
) -> tuple[ExecuteProcess, ExecuteProcess]:
    record_cmd = get_regex_record_cmd(record_config_name, allowlist)
    record_proc = ExecuteProcess(
        cmd=record_cmd,
        condition=UnlessCondition(LaunchConfiguration("override_record_topics")),
    )
    record_override_cmd = get_regex_record_cmd(
        record_config_name,
        LaunchConfiguration("override_topics_regex"),
    )
    record_override_proc = ExecuteProcess(
        cmd=record_override_cmd,
        condition=IfCondition(LaunchConfiguration("override_record_topics")),
    )
    return record_proc, record_override_proc


def get_topic_state_monitor_launch(
    topic_monitor_config: str,
) -> launch.actions.IncludeLaunchDescription:
    # component_state_monitor launch
    component_state_monitor_launch_file = Path(
        get_package_share_directory("component_state_monitor"),
        "launch",
        "component_state_monitor.launch.py",
    )
    topic_monitor_config_path = Path(
        get_package_share_directory("driving_log_replayer"),
        "config",
        topic_monitor_config,
    )
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(
            component_state_monitor_launch_file.as_posix(),
        ),
        launch_arguments={
            "file": topic_monitor_config_path.as_posix(),
            "mode": "logging_simulation",
        }.items(),
    )
