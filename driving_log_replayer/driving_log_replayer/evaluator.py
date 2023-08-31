# Copyright (c) 2023 TIER IV.inc
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

from abc import ABC
from abc import abstractclassmethod
import os
from os.path import expandvars
from typing import Dict
from typing import Optional
from typing import TYPE_CHECKING

from autoware_adapi_v1_msgs.srv import InitializeLocalization
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rosidl_runtime_py import message_to_ordereddict
import simplejson as json
from tf_transformations import euler_from_quaternion
from tier4_localization_msgs.srv import PoseWithCovarianceStamped as PoseWithCovarianceStampedSrv

if TYPE_CHECKING:
    from autoware_adapi_v1_msgs.msg import ResponseStatus


class DLREvaluator(Node, ABC):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("result_json_path", "")

        self._scenario_path = expandvars(
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )
        self._result_json_path = expandvars(
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )

        self._current_time = Time().to_msg()
        self._prev_time = Time().to_msg()
        self._clock_stop_counter = 0

        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self._timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer

    @abstractclassmethod
    def timer_cb(self):
        raise NotImplementedError

    def start_initial_pose(self):
        self._initial_pose_running = False
        self._initial_pose_success = False
        self._initial_pose_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )
        self._map_fit_client = self.create_client(
            PoseWithCovarianceStampedSrv, "/map/map_height_fitter/service"
        )

        while not self._initial_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("initial pose service not available, waiting again...")
        while not self._map_fit_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("map height fitter service not available, waiting again...")

    def map_fit_cb(self, future):
        result = future.result()
        if result is not None:
            if result.success:
                future_init_pose = self._initial_pose_client.call_async(
                    InitializeLocalization.Request(pose=[result.pose_with_covariance])
                )
                future_init_pose.add_done_callback(self.initial_pose_cb)
            else:
                # free self._initial_pose_running when the service call fails
                self._initial_pose_running = False
                self.get_logger().warn("map_height_height service result is fail")
        else:
            # free self._initial_pose_running when the service call fails
            self._initial_pose_running = False
            self.get_logger().error(f"Exception for service: {future.exception()}")

    def initial_pose_cb(self, future):
        result = future.result()
        if result is not None:
            res_status: ResponseStatus = result.status
            self._initial_pose_success = res_status.success
            self.get_logger().info(
                f"initial_pose_success: {self._initial_pose_success}"
            )  # debug msg
        else:
            self.get_logger().error(f"Exception for service: {future.exception()}")
        # free self._initial_pose_running
        self._initial_pose_running = False

    @classmethod
    def get_goal_pose_from_t4_dataset(cls, dataset_path: str) -> PoseStamped:
        ego_pose_json_path = os.path.join(dataset_path, "annotation", "ego_pose.json")
        with open(ego_pose_json_path) as ego_pose_file:
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

    @classmethod
    def transform_stamped_with_euler_angle(cls, transform_stamped: TransformStamped) -> Dict:
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

    @classmethod
    def set_initial_pose(cls, initial_pose: Optional[Dict]) -> Optional[PoseWithCovarianceStamped]:
        try:
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
        except KeyError:
            ros_init_pose: Optional[PoseWithCovarianceStamped] = None
        return ros_init_pose
