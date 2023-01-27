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

import os
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


def get_driving_log_replayer_common_argument():
    """Set and return launch argument.

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

    """
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=""):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg(
        "with_autoware", default_value="true", description="Whether to launch autoware or not"
    )
    add_launch_arg("rviz", default_value="true", description="Whether to display rviz or not")
    add_launch_arg("scenario_path", default_value="/tmp/scenario", description="scenario path")
    add_launch_arg(
        "result_json_path",
        default_value="/tmp/result_json",
        description="result json save path",
    )
    add_launch_arg("play_rate", default_value="1.0", description="ros2 bag play rate")
    add_launch_arg("play_delay", default_value="10.0", description="ros2 bag play delay")
    add_launch_arg("input_bag", description="full path to the input bag")
    add_launch_arg(
        "result_bag_path", default_value="/tmp/result_bag", description="result bag save path"
    )
    add_launch_arg("map_path", description="point cloud and lanelet2 map directory path")
    add_launch_arg("vehicle_model", description="vehicle model name")
    add_launch_arg("sensor_model", description="sensor model name")
    add_launch_arg("vehicle_id", default_value="default", description="vehicle specific ID")

    # additional argument
    add_launch_arg(
        "t4_dataset_path",
        default_value="/tmp/t4_dataset",
        description="full path of t4_dataset",
    )

    add_launch_arg(
        "result_archive_path",
        default_value="/tmp/result_archive",
        description="additional result file",
    )

    return launch_arguments


def get_autoware_launch(
    sensing="true",
    localization="true",
    perception="true",
    planning="false",
    control="false",
    scenario_simulation="false",
):
    # autoware launch
    autoware_launch_file = os.path.join(
        get_package_share_directory("autoware_launch"), "launch", "logging_simulator.launch.xml"
    )
    autoware_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(autoware_launch_file),
        launch_arguments={
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
            "rviz": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_autoware")),
    )
    return autoware_launch


def get_rviz(rviz_config_name: str, package_name: str = "driving_log_replayer"):
    # if you use plugin package set package_name
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_name), "config", rviz_config_name
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return rviz_node


def get_evaluator_node(
    usecase_name: str,
    package_name: str = "driving_log_replayer",
    addition_parameter=None,
    python_node=False,
):
    params = {
        "use_sim_time": True,
        "scenario_path": LaunchConfiguration("scenario_path"),
        "result_json_path": LaunchConfiguration("result_json_path"),
        "t4_dataset_path": LaunchConfiguration("t4_dataset_path"),
        "result_archive_path": LaunchConfiguration("result_archive_path"),
    }
    if addition_parameter is not None and type(addition_parameter) == dict:
        params.update(addition_parameter)

    node_name = usecase_name + "_evaluator_node"
    if python_node:
        node_name += ".py"

    evaluator_node = Node(
        package=package_name,
        namespace="/driving_log_replayer",
        executable=node_name,
        output="screen",
        name=usecase_name + "_evaluator",
        parameters=[params],
    )
    return evaluator_node


def get_evaluator_shutdown(target_node):
    return launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=target_node,
            on_exit=[
                launch.actions.LogInfo(msg="shutdown launch"),
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ],
        )
    )


def get_recorder(record_config_name: str, record_topics: list):
    record_cmd = (
        ["ros2", "bag", "record"]
        + record_topics
        + [
            "-o",
            LaunchConfiguration("result_bag_path"),
            "--qos-profile-overrides-path",
            os.path.join(
                get_package_share_directory("driving_log_replayer"), "config", record_config_name
            ),
        ]
    )
    return ExecuteProcess(cmd=record_cmd)


def get_regex_recorder(record_config_name: str, allowlist: str):
    record_cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        LaunchConfiguration("result_bag_path"),
        "--qos-profile-overrides-path",
        os.path.join(
            get_package_share_directory("driving_log_replayer"), "config", record_config_name
        ),
        "-e",
        allowlist,
    ]
    return ExecuteProcess(cmd=record_cmd)


def get_player(additional_argument=None):
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
    if additional_argument is not None and type(additional_argument) == list:
        play_cmd.extend(additional_argument)
    return ExecuteProcess(
        cmd=["sleep", LaunchConfiguration("play_delay")],
        on_exit=[ExecuteProcess(cmd=play_cmd)],
        output="screen",
    )


def get_evaluator_container(
    usecase_name: str, package_name: str = "driving_log_replayer", addition_parameter=None
):
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
        package=package_name,
        plugin="driving_log_replayer::" + snake_to_pascal(usecase_name) + "EvaluatorComponent",
        namespace="/driving_log_replayer",
        name=usecase_name + "_evaluator",
        parameters=[params],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )
    evaluator_container = ComposableNodeContainer(
        name="DrivingLogReplayer" + snake_to_pascal(usecase_name) + "EvaluatorContainer",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[evaluator_node],
        output="screen",
    )
    return evaluator_container


def add_container_argument(launch_arguments: list):
    launch_arguments.append(DeclareLaunchArgument("use_multithread", default_value="true"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "use_intra_process",
            default_value="false",
            description="use ROS2 component container communication",
        )
    )
    return launch_arguments


def get_container_configuration():
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


def snake_to_pascal(snake_str: str):
    pascal = capwords(snake_str.replace("_", " "))
    pascal = pascal.replace(" ", "")
    return pascal
