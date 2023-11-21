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

from os.path import expandvars
from pathlib import Path
from typing import Any
from typing import Callable
from typing import TYPE_CHECKING

from autoware_adapi_v1_msgs.srv import InitializeLocalization
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import TrafficLight
from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import numpy as np
from pydantic import ValidationError
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
from std_msgs.msg import Header
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from tf_transformations import euler_from_quaternion
from tier4_localization_msgs.srv import PoseWithCovarianceStamped as PoseWithCovarianceStampedSrv
import yaml

from driving_log_replayer.result import PickleWriter
from driving_log_replayer.result import ResultWriter
from driving_log_replayer.scenario import InitialPose
from driving_log_replayer.scenario import load_scenario

if TYPE_CHECKING:
    from autoware_adapi_v1_msgs.msg import ResponseStatus


class DLREvaluator(Node):
    COUNT_SHUTDOWN_NODE = 5

    def __init__(self, name: str, scenario_class: Callable, result_class: Callable) -> None:
        super().__init__(name)
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("result_json_path", "")

        self._scenario_path = expandvars(
            self.get_parameter("scenario_path").get_parameter_value().string_value,
        )
        self._result_json_path = expandvars(
            self.get_parameter("result_json_path").get_parameter_value().string_value,
        )

        self._scenario = None
        try:
            self._scenario = load_scenario(Path(self._scenario_path), scenario_class)
            self._result_writer = ResultWriter(
                self._result_json_path,
                self.get_clock(),
                self._scenario.Evaluation.Conditions,
            )
            self._result = result_class(self._scenario.Evaluation.Conditions)
        except (FileNotFoundError, PermissionError, yaml.YAMLError, ValidationError) as e:
            self.get_logger().error(f"An error occurred while loading the scenario. {e}")
            self._result_writer = ResultWriter(
                self._result_json_path,
                self.get_clock(),
                {},
            )
            error_dict = {
                "Result": {"Success": False, "Summary": "ScenarioFormatError"},
                "Stamp": {"System": 0.0},
                "Frame": {"ErrorMsg": e.__str__()},
            }
            self._result_writer.write_line(error_dict)
            self._result_writer.close()
            rclpy.shutdown()

        self.set_t4_dataset()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._initial_pose = self.set_initial_pose()
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

    def set_t4_dataset(self) -> None:
        if self._scenario.ScenarioFormatVersion != "3.0.0":
            return
        self.declare_parameter("t4_dataset_path", "")
        self.declare_parameter("result_archive_path", "")

        result_archive_path = Path(
            expandvars(
                self.get_parameter("result_archive_path").get_parameter_value().string_value,
            ),
        )
        result_archive_path.mkdir(exist_ok=True)

        self._pkl_path = result_archive_path.joinpath("scene_result.pkl").as_posix()
        self._t4_dataset_paths = [
            expandvars(self.get_parameter("t4_dataset_path").get_parameter_value().string_value),
        ]
        self._perception_eval_log_path = result_archive_path.parent.joinpath(
            "perception_eval_log",
        ).as_posix()

    def timer_cb(
        self,
        *,
        register_loop_func: Callable | None = None,
        register_shutdown_func: Callable | None = None,
    ) -> None:
        self._current_time = self.get_clock().now().to_msg()
        # to debug callback use: self.get_logger().error(f"time: {self._current_time.sec}.{self._current_time.nanosec}")
        if self._current_time.sec <= 0:
            return
        if register_loop_func is not None:
            register_loop_func()
        if self._initial_pose is not None:
            self.call_initialpose_service()
        self._clock_stop_counter = (
            self._clock_stop_counter + 1 if self._current_time == self._prev_time else 0
        )
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
            InitializeLocalization,
            "/api/localization/initialize",
        )
        self._map_fit_client = self.create_client(
            PoseWithCovarianceStampedSrv,
            "/map/map_height_fitter/service",
        )

        while not self._initial_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("initial pose service not available, waiting again...")
        while not self._map_fit_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("map height fitter service not available, waiting again...")

    def call_initialpose_service(self) -> None:
        if not self._initial_pose_success and not self._initial_pose_running:
            self.get_logger().info(
                f"call initial_pose time: {self._current_time.sec}.{self._current_time.nanosec}",
            )
            self._initial_pose_running = True
            self._initial_pose.header.stamp = self._current_time
            future_map_fit = self._map_fit_client.call_async(
                PoseWithCovarianceStampedSrv.Request(pose_with_covariance=self._initial_pose),
            )
            future_map_fit.add_done_callback(self.map_fit_cb)

    def map_fit_cb(self, future) -> None:  # noqa
        result = future.result()
        if result is not None:
            if result.success:
                future_init_pose = self._initial_pose_client.call_async(
                    InitializeLocalization.Request(pose=[result.pose_with_covariance]),
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

    def initial_pose_cb(self, future) -> None:  # noqa
        result = future.result()
        if result is not None:
            res_status: ResponseStatus = result.status
            self._initial_pose_success = res_status.success
            self.get_logger().info(
                f"initial_pose_success: {self._initial_pose_success}",
            )  # debug msg
        else:
            self.get_logger().error(f"Exception for service: {future.exception()}")
        # free self._initial_pose_running
        self._initial_pose_running = False

    def lookup_transform(self, stamp: Stamp) -> TransformStamped:
        try:
            return self._tf_buffer.lookup_transform(
                "map",
                "base_link",
                stamp,
                Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            return TransformStamped()

    def save_pkl(self, save_object: Any) -> None:
        PickleWriter(self._pkl_path, save_object)

    @classmethod
    def get_goal_pose_from_t4_dataset(cls, dataset_path: str) -> PoseStamped:
        ego_pose_json_path = Path(dataset_path, "annotation", "ego_pose.json")
        with ego_pose_json_path.open() as ego_pose_file:
            ego_pose_json = json.load(ego_pose_file)
            last_ego_pose = ego_pose_json[-1]
            pose = Pose(
                position=Point(
                    x=last_ego_pose["translation"][0],
                    y=last_ego_pose["translation"][1],
                    z=last_ego_pose["translation"][2],
                ),
                orientation=Quaternion(
                    x=last_ego_pose["rotation"][1],
                    y=last_ego_pose["rotation"][2],
                    z=last_ego_pose["rotation"][3],
                    w=last_ego_pose["rotation"][0],
                ),
            )
            return PoseStamped(header=Header(frame_id="map"), pose=pose)

    @classmethod
    def transform_stamped_with_euler_angle(cls, transform_stamped: TransformStamped) -> dict:
        tf_euler = message_to_ordereddict(transform_stamped)
        euler_angle = euler_from_quaternion(
            [
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w,
            ],
        )
        tf_euler["rotation_euler"] = {
            "roll": euler_angle[0],
            "pitch": euler_angle[1],
            "yaw": euler_angle[2],
        }
        return tf_euler

    def set_initial_pose(self) -> PoseWithCovarianceStamped | None:
        if not hasattr(self._scenario.Evaluation, "InitialPose"):
            return None
        if self._scenario.Evaluation.InitialPose is None:
            return None
        initial_pose: InitialPose = self._scenario.Evaluation.InitialPose
        covariance = np.array(
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
            ],
        )
        pose = PoseWithCovariance(
            pose=Pose(
                position=Point(**initial_pose.position.model_dump()),
                orientation=Quaternion(**initial_pose.orientation.model_dump()),
            ),
            covariance=covariance,
        )
        return PoseWithCovarianceStamped(header=Header(frame_id="map"), pose=pose)

    @classmethod
    def get_perception_label_str(cls, classification: ObjectClassification) -> str:
        label_str_dict = {
            ObjectClassification.UNKNOWN: "unknown",
            ObjectClassification.CAR: "car",
            ObjectClassification.TRUCK: "truck",
            ObjectClassification.BUS: "bus",
            ObjectClassification.TRAILER: "trailer",
            ObjectClassification.MOTORCYCLE: "motorbike",
            ObjectClassification.BICYCLE: "bicycle",
            ObjectClassification.PEDESTRIAN: "pedestrian",
        }
        return label_str_dict.get(classification.label, "other")

    @classmethod
    def get_most_probable_classification(
        cls,
        array_classification: list[ObjectClassification],
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
        lights: list[TrafficLight],
    ) -> TrafficLight:
        highest_probability = 0.0
        highest_light = None
        for light in lights:
            if light.confidence >= highest_probability:
                highest_probability = light.confidence
                highest_light = light
        return highest_light


def evaluator_main(func: Callable) -> Callable:
    def wrapper() -> None:
        rclpy.init()
        executor = MultiThreadedExecutor()
        evaluator = func()
        executor.add_node(evaluator)
        executor.spin()
        evaluator.destroy_node()
        rclpy.shutdown()

    return wrapper
