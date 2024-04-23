import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import driving_log_replayer.launch_common as cmn
from driving_log_replayer.shutdown_once import ShutdownOnce

RECORD_TOPIC_REGEX = """^/clock$\
|^/tf$\
|^/driving_log_replayer/.*\
"""


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    autoware_launch = cmn.get_autoware_launch(
        planning="false",
        localization="false",
        control="false",
        scenario_simulation="true",
        perception_mode="lidar",
    )
    rviz_node = cmn.get_rviz("ground_segmentation.rviz")
    evaluator_node = cmn.get_evaluator_node(
        "ground_segmentation",
        addition_parameter={"vehicle_model": LaunchConfiguration("vehicle_model")},
    )
    # evaluator_node_sub = Node(
    #     package="driving_log_replayer",
    #     namespace="/driving_log_replayer",
    #     executable="ground_segmentation_evaluator_node",
    #     output="screen",
    #     name="ground_segmentation_sub",
    #     on_exit=ShutdownOnce(),
    # )
    player = cmn.get_player()

    recorder, recorder_override = cmn.get_regex_recorders(
        "perception.qos.yaml",
        RECORD_TOPIC_REGEX,
    )

    return launch.LaunchDescription(
        [
            *launch_arguments,
            rviz_node,
            autoware_launch,
            evaluator_node,
            # evaluator_node_sub,
            player,
            recorder,
            recorder_override,
        ]
    )
