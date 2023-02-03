#!/usr/bin/env python3

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
from typing import Dict
from typing import Optional

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
import numpy as np
from rosidl_runtime_py import message_to_ordereddict
import simplejson as json
from tf_transformations import euler_from_quaternion


def get_goal_pose_from_t4_dataset(dataset_path: str) -> PoseStamped:
    ego_pose_json_path = os.path.join(dataset_path, "annotation", "ego_pose.json")
    with open(ego_pose_json_path, "r") as ego_pose_file:
        ego_pose_json = json.load(ego_pose_file)
        last_ego_pose = ego_pose_json[-1]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = last_ego_pose["translation"][0]
        goal_pose.pose.position.y = last_ego_pose["translation"][1]
        goal_pose.pose.position.z = last_ego_pose["translation"][2]
        goal_pose.pose.orientation.x = last_ego_pose["rotation"][1]
        goal_pose.pose.orientation.y = last_ego_pose["rotation"][2]
        goal_pose.pose.orientation.z = last_ego_pose["rotation"][3]
        goal_pose.pose.orientation.w = last_ego_pose["rotation"][0]
        return goal_pose


def transform_stamped_with_euler_angle(transform_stamped: TransformStamped) -> Dict:
    tf_euler = message_to_ordereddict(transform_stamped)
    euler_angle = euler_from_quaternion(
        [
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w,
        ]
    )
    tf_euler["rotation_euler"] = {
        "roll": euler_angle[0],
        "pitch": euler_angle[1],
        "yaw": euler_angle[2],
    }
    return tf_euler


def set_initial_pose(initial_pose: Optional[Dict]) -> Optional[PoseWithCovarianceStamped]:
    ros_init_pose: Optional[PoseWithCovarianceStamped] = None
    if initial_pose is not None:
        ros_init_pose = PoseWithCovarianceStamped()
        ros_init_pose.header.frame_id = "map"
        ros_init_pose.pose.pose.position.x = float(initial_pose["position"]["x"])
        ros_init_pose.pose.pose.position.y = float(initial_pose["position"]["y"])
        ros_init_pose.pose.pose.position.z = float(initial_pose["position"]["z"])
        ros_init_pose.pose.pose.orientation.x = float(initial_pose["orientation"]["x"])
        ros_init_pose.pose.pose.orientation.y = float(initial_pose["orientation"]["y"])
        ros_init_pose.pose.pose.orientation.z = float(initial_pose["orientation"]["z"])
        ros_init_pose.pose.pose.orientation.w = float(initial_pose["orientation"]["w"])
        ros_init_pose.pose.covariance = np.array(
            [
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.06853892326654787,
            ]
        )
    return ros_init_pose
