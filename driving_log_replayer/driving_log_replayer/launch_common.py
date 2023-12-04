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
from string import capwords

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from driving_log_replayer.shutdown_once import ShutdownOnce


def get_driving_log_replayer_common_argument() -> list:
    """
    Set and return launch argument.

    with_autoware
    rviz
    scenario_path
    result_json_path
    play_rate
    play_delay
    input_bag
    result_bag_path
    map_path
    vehicle_model
    sensor_model
    vehicle_id
    t4_dataset_path
    result_archive_path
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
    add_launch_arg("rviz", default_value="true", description="Whether to display rviz or not")
    add_launch_arg("scenario_path", description="scenario path")
    add_launch_arg(
        "result_json_path",
        description="result json save path",
    )
    add_launch_arg("play_rate", default_value="1.0", description="ros2 bag play rate")
    add_launch_arg("play_delay", default_value="10.0", description="ros2 bag play delay")
    add_launch_arg("input_bag", description="full path to the input bag")
    add_launch_arg(
        "result_bag_path",
        description="result bag save path",
    )
    add_launch_arg("map_path", description="point cloud and lanelet2 map directory path")
    add_launch_arg("vehicle_model", description="vehicle model name")
    add_launch_arg("sensor_model", description="sensor model name")
    add_launch_arg("vehicle_id", default_value="default", description="vehicle specific ID")

    # additional argument
    add_launch_arg(
        "t4_dataset_path",
        default_value="/opt/autoware/t4_dataset",
        description="full path of t4_dataset",
    )
    add_launch_arg(
        "result_archive_path",
        default_value="/opt/autoware/result_archive",
        description="additional result file",
    )
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


def get_autoware_launch(
    sensing: str = "true",
    localization: str = "true",
    perception: str = "true",
    planning: str = "false",
    control: str = "false",
    scenario_simulation: str = "false",
    pose_source: str = "ndt",
    twist_source: str = "gyro_odom",
    perception_mode: str | None = None,
) -> launch.actions.IncludeLaunchDescription:
    # autoware launch
    autoware_launch_file = Path(
        get_package_share_directory("autoware_launch"),
        "launch",
        "logging_simulator.launch.xml",
    )
    launch_args = {
        "map_path": LaunchConfiguration("map_path"),
        "vehicle_model": LaunchConfiguration("vehicle_model"),
        "sensor_model": LaunchConfiguration("sensor_model"),
        "vehicle_id": LaunchConfiguration("vehicle_id"),
        "launch_vehicle_interface": "true",
        "sensing": sensing,
        "localization": localization,
        "perception": perception,
        "planning": planning,
        "control": control,
        "scenario_simulation": scenario_simulation,
        "pose_source": pose_source,
        "twist_source": twist_source,
        "rviz": "false",
    }
    if isinstance(perception_mode, str):
        launch_args["perception_mode"] = perception_mode
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(
            autoware_launch_file.as_posix(),
        ),
        launch_arguments=launch_args.items(),
        condition=IfCondition(LaunchConfiguration("with_autoware")),
    )


def get_map_height_fitter(launch_service: str = "true") -> launch.actions.IncludeLaunchDescription:
    # map_height_fitter launch
    fitter_launch_file = Path(
        get_package_share_directory("map_height_fitter"),
        "launch",
        "map_height_fitter.launch.xml",
    )
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(fitter_launch_file.as_posix()),
        condition=IfCondition(launch_service),
    )


def get_rviz(rviz_config_name: str) -> Node:
    rviz_config_dir = Path(
        get_package_share_directory("driving_log_replayer"),
        "config",
        rviz_config_name,
    )
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir.as_posix()],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )


def get_evaluator_node(
    usecase_name: str,
    addition_parameter: dict | None = None,
) -> Node:
    params = {
        "use_sim_time": True,
        "scenario_path": LaunchConfiguration("scenario_path"),
        "result_json_path": LaunchConfiguration("result_json_path"),
        "t4_dataset_path": LaunchConfiguration("t4_dataset_path"),
        "result_archive_path": LaunchConfiguration("result_archive_path"),
    }
    if addition_parameter is not None and isinstance(addition_parameter, dict):
        params.update(addition_parameter)

    node_name = usecase_name + "_evaluator_node.py"

    return Node(
        package="driving_log_replayer",
        namespace="/driving_log_replayer",
        executable=node_name,
        output="screen",
        name=usecase_name + "_evaluator",
        parameters=[params],
        on_exit=ShutdownOnce(),
    )


def create_regex_record_cmd(record_config_name: str, allowlist: str) -> list:
    return [
        "ros2",
        "bag",
        "record",
        "-o",
        LaunchConfiguration("result_bag_path"),
        "--qos-profile-overrides-path",
        Path(
            get_package_share_directory("driving_log_replayer"),
            "config",
            record_config_name,
        ).as_posix(),
        "-e",
        allowlist,
    ]


def get_regex_recorder(record_config_name: str, allowlist: str) -> ExecuteProcess:
    record_cmd = create_regex_record_cmd(record_config_name, allowlist)
    return ExecuteProcess(cmd=record_cmd)


def get_regex_recorders(
    record_config_name: str,
    allowlist: str,
) -> tuple[ExecuteProcess, ExecuteProcess]:
    record_cmd = create_regex_record_cmd(record_config_name, allowlist)
    record_proc = ExecuteProcess(
        cmd=record_cmd,
        condition=UnlessCondition(LaunchConfiguration("override_record_topics")),
    )
    record_override_cmd = create_regex_record_cmd(
        record_config_name,
        LaunchConfiguration("override_topics_regex"),
    )
    record_override_proc = ExecuteProcess(
        cmd=record_override_cmd,
        condition=IfCondition(LaunchConfiguration("override_record_topics")),
    )
    return record_proc, record_override_proc


def get_player_cmd(additional_argument: list | None = None) -> list:
    play_cmd = [
        "ros2",
        "bag",
        "play",
        LaunchConfiguration("input_bag"),
        "--rate",
        LaunchConfiguration("play_rate"),
        "--clock",
        "200",
    ]
    if additional_argument is not None and isinstance(additional_argument, list):
        play_cmd.extend(additional_argument)
    return play_cmd


def get_player(
    additional_argument: list | None = None,
    condition: IfCondition | None = None,
) -> ExecuteProcess:
    play_cmd = get_player_cmd(additional_argument)
    if condition is not None:
        return ExecuteProcess(
            cmd=["sleep", LaunchConfiguration("play_delay")],
            on_exit=[ExecuteProcess(cmd=play_cmd)],
            output="screen",
            condition=condition,
        )
    return ExecuteProcess(
        cmd=["sleep", LaunchConfiguration("play_delay")],
        on_exit=[ExecuteProcess(cmd=play_cmd)],
        output="screen",
    )


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


def get_evaluator_container(
    usecase_name: str,
    addition_parameter: dict | None = None,
) -> ComposableNodeContainer:
    params = {
        "use_sim_time": True,
        "scenario_path": LaunchConfiguration("scenario_path"),
        "result_json_path": LaunchConfiguration("result_json_path"),
        "t4_dataset_path": LaunchConfiguration("t4_dataset_path"),
        "result_archive_path": LaunchConfiguration("result_archive_path"),
    }
    if addition_parameter is not None and type(addition_parameter) == dict:
        params.update(addition_parameter)

    evaluator_node = ComposableNode(
        package="driving_log_replayer",
        plugin="driving_log_replayer::" + snake_to_pascal(usecase_name) + "EvaluatorComponent",
        namespace="/driving_log_replayer",
        name=usecase_name + "_evaluator",
        parameters=[params],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )
    return ComposableNodeContainer(
        name="DrivingLogReplayer" + snake_to_pascal(usecase_name) + "EvaluatorContainer",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[evaluator_node],
        output="screen",
    )


def add_container_argument(launch_arguments: list) -> list:
    launch_arguments.append(DeclareLaunchArgument("use_multithread", default_value="true"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "use_intra_process",
            default_value="false",
            description="use ROS 2 component container communication",
        ),
    )
    return launch_arguments


def get_container_configuration() -> tuple[SetLaunchConfiguration, SetLaunchConfiguration]:
    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )
    return [set_container_executable, set_container_mt_executable]


def snake_to_pascal(snake_str: str) -> str:
    pascal = capwords(snake_str.replace("_", " "))
    return pascal.replace(" ", "")
