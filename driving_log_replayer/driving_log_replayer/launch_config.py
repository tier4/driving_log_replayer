# Copyright (c) 2024 TIER IV.inc
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

from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue

LOCALIZATION_RECORD_TOPIC = """^/tf$\
|^/diagnostics$\
|^/localization/pose_estimator/transform_probability$\
|^/localization/pose_estimator/nearest_voxel_transformation_likelihood$\
|^/localization/pose_estimator/pose$\
|^/localization/kinematic_state$\
|^/localization/util/downsample/pointcloud$\
|^/localization/pose_estimator/points_aligned$\
|^/driving_log_replayer/.*\
"""

LOCALIZATION_AUTOWARE_ARGS = {
    "perception": "false",
    "planning": "false",
    "control": "false",
    "pose_source": "ndt",
    "twist_source": "gyro_odom",
}

LOCALIZATION_NODE_PARAMS = {}

EAGLEYE_RECORD_TOPIC = """^/tf$\
|^/diagnostics$"\
|^/localization/kinematic_state$\
|^/localization/pose_estimator/pose$\
|^/localization/util/downsample/pointcloud$\
|^/localization/pose_estimator/points_aligned$\
"""

EAGLEYE_AUTOWARE_ARGS = {
    "perception": "false",
    "planning": "false",
    "control": "false",
    "pose_source": "eagleye",
    "twist_source": "eagleye",
}

EAGLEYE_NODE_PARAMS = {}


AR_TAG_BASED_LOCALIZER_RECORD_TOPIC = """^/tf$\
|^/diagnostics$"\
|^/localization/kinematic_state$\
"""

AR_TAG_BASED_LOCALIZER_AUTOWARE_ARGS = {
    "perception": "false",
    "planning": "false",
    "control": "false",
    "pose_source": "artag",
    "twist_source": "gyro_odom",
}

AR_TAG_BASED_LOCALIZER_NODE_PARAMS = {}

YABLOC_RECORD_TOPIC = """^/tf$\
|^/diagnostics$\
|^/localization/pose_estimator/pose$\
|^/localization/kinematic_state$\
|^/localization/util/downsample/pointcloud$\
|^/localization/pose_estimator/points_aligned$\
"""

YABLOC_AUTOWARE_ARGS = {
    "perception": "false",
    "planning": "false",
    "control": "false",
    "pose_source": "yabloc",
    "twist_source": "gyro_odom",
}

YABLOC_NODE_PARAMS = {}

PERCEPTION_RECORD_TOPIC = """^/tf$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/driving_log_replayer/.*\
|^/sensing/camera/.*\
"""


PERCEPTION_AUTOWARE_ARGS = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

PERCEPTION_NODE_PARAMS = {}

OBSTACLE_SEGMENTATION_RECORD_TOPIC = """^/tf$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/planning/scenario_planning/trajectory$\
|^/planning/scenario_planning/status/stop_reasons$\
|^/driving_log_replayer/.*\
"""

OBSTACLE_SEGMENTATION_AUTOWARE_ARGS = {
    "localization": "false",
    "planning": "true",
    "control": "false",
    "scenario_simulation": "true",
}

OBSTACLE_SEGMENTATION_NODE_PARAMS = {
    "vehicle_model": LaunchConfiguration("vehicle_model"),
    "map_path": LaunchConfiguration("map_path"),
}

TRAFFIC_LIGHT_RECORD_TOPIC = """^/tf$\
|^/sensing/camera/camera[67]/image_raw/compressed$\
|^/perception/.*/traffic_signals$\
|^/driving_log_replayer/.*\
"""

TRAFFIC_LIGHT_AUTOWARE_ARGS = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

TRAFFIC_LIGHT_NODE_PARAMS = {"map_path": LaunchConfiguration("map_path")}

PERFORMANCE_DIAG_RECORD_TOPIC = """^/tf$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/diagnostics$\
|^/sensing/lidar/.*/blockage_diag/debug/blockage_mask_image$\
|^/sensing/lidar/.*/pointcloud_raw_ex$\
|^/driving_log_replayer/.*\
"""

PERFORMANCE_DIAG_AUTOWARE_ARGS = {
    "perception": "false",
    "planning": "false",
    "control": "false",
}

PERFORMANCE_DIAG_NODE_PARAMS = {}

PERCEPTION_2D_RECORD_TOPIC = """^/tf$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/sensing/camera/.*\
|^/driving_log_replayer/.*\
"""

PERCEPTION_2D_AUTOWARE_ARGS = {
    "localization": "false",
    "planning": "false",
    "control": "false",
    "perception_mode": "camera_lidar_fusion",
}

PERCEPTION_2D_NODE_PARAMS = {}

ANNOTATIONLESS_PERCEPTION_RECORD_TOPIC = """^/tf$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/diagnostic/perception_online_evaluator/.*\
"""

ANNOTATIONLESS_PERCEPTION_AUTOWARE_ARGS = {
    "localization": "false",
    "use_perception_online_evaluator": "true",
}

ANNOTATIONLESS_PERCEPTION_NODE_PARAMS = {
    "annotationless_threshold_file": LaunchConfiguration("annotationless_threshold_file"),
    # annotationless_pass_range is json format string. Avoid interpreting json format strings as dict
    # [ERROR] [launch]: Caught exception in launch (see debug for traceback): Allowed value types are bytes, bool, int, float, str, Sequence[bool], Sequence[int], Sequence[float], Sequence[str].
    # Got <class 'dict'>.If the parameter is meant to be a string, try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
    "annotationless_pass_range": ParameterValue(
        LaunchConfiguration("annotationless_pass_range"),
        value_type=str,
    ),
}

dlr_config = {
    "localization": {
        "record": LOCALIZATION_RECORD_TOPIC,
        "autoware": LOCALIZATION_AUTOWARE_ARGS,
        "node": LOCALIZATION_NODE_PARAMS,
    },
    "eagleye": {
        "record": EAGLEYE_RECORD_TOPIC,
        "autoware": EAGLEYE_AUTOWARE_ARGS,
        "node": EAGLEYE_NODE_PARAMS,
    },
    "ar_tag_based_localizer": {
        "record": AR_TAG_BASED_LOCALIZER_RECORD_TOPIC,
        "autoware": AR_TAG_BASED_LOCALIZER_AUTOWARE_ARGS,
        "node": AR_TAG_BASED_LOCALIZER_NODE_PARAMS,
    },
    "yabloc": {
        "record": YABLOC_RECORD_TOPIC,
        "autoware": YABLOC_AUTOWARE_ARGS,
        "node": YABLOC_NODE_PARAMS,
    },
    "perception": {
        "record": PERCEPTION_RECORD_TOPIC,
        "autoware": PERCEPTION_AUTOWARE_ARGS,
        "node": PERCEPTION_NODE_PARAMS,
    },
    "obstacle_segmentation": {
        "record": OBSTACLE_SEGMENTATION_RECORD_TOPIC,
        "autoware": OBSTACLE_SEGMENTATION_AUTOWARE_ARGS,
        "node": OBSTACLE_SEGMENTATION_NODE_PARAMS,
    },
    "traffic_light": {
        "record": TRAFFIC_LIGHT_RECORD_TOPIC,
        "autoware": TRAFFIC_LIGHT_AUTOWARE_ARGS,
        "node": TRAFFIC_LIGHT_NODE_PARAMS,
    },
    "performance_diag": {
        "record": PERFORMANCE_DIAG_RECORD_TOPIC,
        "autoware": PERFORMANCE_DIAG_AUTOWARE_ARGS,
        "node": PERFORMANCE_DIAG_NODE_PARAMS,
    },
    "perception_2d": {
        "record": PERCEPTION_2D_RECORD_TOPIC,
        "autoware": PERCEPTION_AUTOWARE_ARGS,
        "node": PERCEPTION_2D_NODE_PARAMS,
    },
}
