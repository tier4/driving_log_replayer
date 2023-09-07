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
from abc import abstractmethod
from collections.abc import Callable
import os
from os.path import expandvars
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import TYPE_CHECKING

from autoware_adapi_v1_msgs.srv import InitializeLocalization
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import TrafficLight
from builtin_interfaces.msg import Time as Stamp
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
from rclpy.time import Duration
from rclpy.time import Time
from rosidl_runtime_py import message_to_ordereddict
import simplejson as json
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from tf_transformations import euler_from_quaternion
from tier4_localization_msgs.srv import PoseWithCovarianceStamped as PoseWithCovarianceStampedSrv
import yaml

from driving_log_replayer.result import PickleWriter
from driving_log_replayer.result import ResultWriter

if TYPE_CHECKING:
    from autoware_adapi_v1_msgs.msg import ResponseStatus


class DLREvaluator(Node, ABC):
    COUNT_SHUTDOWN_NODE = 5

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

        self._scenario_yaml_obj = None
        try:
            with open(self._scenario_path) as scenario_file:
                self._scenario_yaml_obj = yaml.safe_load(scenario_file)
        except (FileNotFoundError, PermissionError, yaml.YAMLError) as e:
            self.get_logger().error(f"An error occurred while loading the scenario. {e}")
            rclpy.shutdown()

        self._condition = self._scenario_yaml_obj["Evaluation"].get("Conditions", {})
        self._result_writer = ResultWriter(
            self._result_json_path, self.get_clock(), self._condition
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._initial_pose = DLREvaluator.set_initial_pose(
            self._scenario_yaml_obj["Evaluation"].get("InitialPose", None)
        )
        self.start_initialpose_service()

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

    def use_t4_dataset(self) -> None:
        self.declare_parameter("t4_dataset_path", "")
        self.declare_parameter("result_archive_path", "")

        result_archive_path = Path(
            expandvars(self.get_parameter("result_archive_path").get_parameter_value().string_value)
        )
        result_archive_path.mkdir(exist_ok=True)

        self._pkl_path = result_archive_path.joinpath("scene_result.pkl").as_posix()
        self._t4_dataset_paths = [
            expandvars(self.get_parameter("t4_dataset_path").get_parameter_value().string_value)
        ]
        self._perception_eval_log_path = result_archive_path.parent.joinpath(
            "perception_eval_log"
        ).as_posix()

    def timer_cb(
        self,
        *,
        register_startup_func: Optional[Callable] = None,
        register_shutdown_func: Optional[Callable] = None,
    ) -> None:
        self._current_time = self.get_clock().now().to_msg()
        # self.get_logger().error(f"time: {self.__current_time.sec}.{self.__current_time.nanosec}")
        if self._current_time.sec > 0:
            if register_startup_func is not None:
                register_startup_func()
            if self._initial_pose is not None:
                self.call_initialpose_service()
            if self._current_time == self._prev_time:
                self._clock_stop_counter += 1
            else:
                self._clock_stop_counter = 0
            self._prev_time = self._current_time
            if self._clock_stop_counter >= DLREvaluator.COUNT_SHUTDOWN_NODE:
                if register_shutdown_func is not None:
                    register_shutdown_func()
                self._result_writer.close()
                rclpy.shutdown()

    def start_initialpose_service(self) -> None:
        if self._initial_pose is None:
            return
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

    def call_initialpose_service(self) -> None:
        if not self._initial_pose_success and not self._initial_pose_running:
            self.get_logger().info(
                f"call initial_pose time: {self._current_time.sec}.{self._current_time.nanosec}"
            )
            self._initial_pose_running = True
            self._initial_pose.header.stamp = self._current_time
            future_map_fit = self._map_fit_client.call_async(
                PoseWithCovarianceStampedSrv.Request(pose_with_covariance=self._initial_pose)
            )
            future_map_fit.add_done_callback(self.map_fit_cb)

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

    def lookup_transform(self, stamp: Stamp) -> TransformStamped:
        try:
            return self._tf_buffer.lookup_transform(
                "map", "base_link", stamp, Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            return TransformStamped()

    def save_pkl(self, save_object: Any):
        PickleWriter(self._pkl_path, save_object)

    @abstractmethod
    def check_scenario(self) -> None:
        """Check self._scenario_yaml_obj and if has error shutdown."""
        if self._scenario_yaml_obj is None:
            rclpy.shutdown()

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
        if initial_pose is None:
            return None
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
            return None
        else:
            return ros_init_pose

    @classmethod
    def get_perception_label_str(cls, classification: ObjectClassification) -> str:
        if classification.label == ObjectClassification.UNKNOWN:
            return "unknown"
        if classification.label == ObjectClassification.CAR:
            return "car"
        if classification.label == ObjectClassification.TRUCK:
            return "truck"
        if classification.label == ObjectClassification.BUS:
            return "bus"
        if classification.label == ObjectClassification.TRAILER:
            # not implemented in iv
            return "trailer"
        if classification.label == ObjectClassification.MOTORCYCLE:
            # iv: motorbike, auto: motorbike
            return "motorbike"
        if classification.label == ObjectClassification.BICYCLE:
            return "bicycle"
        if classification.label == ObjectClassification.PEDESTRIAN:
            return "pedestrian"
        return "other"

    @classmethod
    def get_most_probable_classification(
        cls,
        array_classification: List[ObjectClassification],
    ) -> ObjectClassification:
        highest_probability = 0.0
        highest_classification = None
        for classification in array_classification:
            if classification.probability >= highest_probability:
                highest_probability = classification.probability
                highest_classification = classification
        return highest_classification

    @classmethod
    def get_traffic_light_label_str(cls, light: TrafficLight) -> str:
        if light.color == TrafficLight.RED:
            return "red"
        if light.color == TrafficLight.AMBER:
            return "yellow"
        if light.color == TrafficLight.GREEN:
            return "green"
        return "unknown"

    @classmethod
    def get_most_probable_signal(
        cls,
        lights: List[TrafficLight],
    ) -> TrafficLight:
        highest_probability = 0.0
        highest_light = None
        for light in lights:
            if light.confidence >= highest_probability:
                highest_probability = light.confidence
                highest_light = light
        return highest_light


def evaluator_main(func):
    def wrapper():
        rclpy.init()
        executor = MultiThreadedExecutor()
        evaluator = func()
        executor.add_node(evaluator)
        executor.spin()
        evaluator.destroy_node()
        rclpy.shutdown()

    return wrapper
