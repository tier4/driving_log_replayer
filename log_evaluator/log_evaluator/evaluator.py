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

from collections.abc import Callable
from pathlib import Path
from typing import Any
from typing import TYPE_CHECKING

from autoware_perception_msgs.msg import ObjectClassification
from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
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
from rclpy.task import Future
from rclpy.time import Duration
from rclpy.time import Time
from rosidl_runtime_py import message_to_ordereddict
from std_msgs.msg import Header
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from tf_transformations import euler_from_quaternion
from tier4_localization_msgs.srv import InitializeLocalization
from tier4_localization_msgs.srv import PoseWithCovarianceStamped as PoseWithCovarianceStampedSrv
import yaml

from log_evaluator.result import PickleWriter
from log_evaluator.result import ResultWriter
from log_evaluator.scenario import InitialPose
from log_evaluator.scenario import load_scenario

if TYPE_CHECKING:
    from autoware_common_msgs.msg import ResponseStatus


class LogEvaluator(Node):
    COUNT_SHUTDOWN_NODE = 5

    def __init__(self, name: str, scenario_class: Callable, result_class: Callable) -> None:
        super().__init__(name)
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("t4_dataset_path", "")
        self.declare_parameter("result_json_path", "")
        self.declare_parameter("result_archive_path", "")
        self.declare_parameter("dataset_index", "")

        self._scenario_path = self.get_parameter("scenario_path").get_parameter_value().string_value
        self._t4_dataset_paths = [
            self.get_parameter("t4_dataset_path").get_parameter_value().string_value,
        ]
        self._result_json_path = (
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )
        self._result_archive_path = Path(
            self.get_parameter("result_archive_path").get_parameter_value().string_value,
        )
        self._dataset_index = (
            self.get_parameter("dataset_index").get_parameter_value().integer_value
        )
        self._result_archive_path.mkdir(exist_ok=True)
        self._perception_eval_log_path = self._result_archive_path.parent.joinpath(
            "perception_eval_log",
        ).as_posix()

        self._scenario = None
        try:
            self._scenario = load_scenario(Path(self._scenario_path), scenario_class)
            evaluation_condition = {}
            if (
                hasattr(self._scenario.Evaluation, "Conditions")
                and self._scenario.Evaluation.Conditions is not None
            ):
                evaluation_condition = self._scenario.Evaluation.Conditions
            self._result_writer = ResultWriter(
                self._result_json_path,
                self.get_clock(),
                evaluation_condition,
            )
            self._result = result_class(evaluation_condition)
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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # initial pose estimation
        self._initial_pose_running: bool = False
        self._initial_pose_success: bool = False
        self._initial_pose, self._initial_pose_method = self.set_initial_pose()
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

    def timer_cb(
        self,
        *,
        register_loop_func: Callable | None = None,
        register_shutdown_func: Callable | None = None,
    ) -> None:
        self._current_time = self.get_clock().now().to_msg()
        # to debug callback use: self.get_logger().error(f"time: {self._current_time.sec}.{self._current_time.nanosec}")
        if self._current_time.sec <= 0:  # Stop PLAYER after standing for 1 second.
            return
        if register_loop_func is not None:
            register_loop_func()
        if self._initial_pose is not None:
            self.call_initialpose_service()
        self._clock_stop_counter = (
            self._clock_stop_counter + 1 if self._current_time == self._prev_time else 0
        )
        self._prev_time = self._current_time
        if self._clock_stop_counter >= LogEvaluator.COUNT_SHUTDOWN_NODE:
            if register_shutdown_func is not None:
                register_shutdown_func()
            self._result_writer.close()
            rclpy.shutdown()

    def start_initialpose_service(self) -> None:
        if self._initial_pose is None:
            return
        self._initial_pose_client = self.create_client(
            InitializeLocalization,
            "/localization/initialize",
        )
        while not self._initial_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("initial pose service not available, waiting again...")

        if self._initial_pose_method == InitializeLocalization.Request.AUTO:
            self._map_fit_client = self.create_client(
                PoseWithCovarianceStampedSrv,
                "/map/map_height_fitter/service",
            )
            while not self._map_fit_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning(
                    "map height fitter service not available, waiting again...",
                )

    def call_initialpose_service(self) -> None:
        if self._initial_pose is None or self._initial_pose_success or self._initial_pose_running:
            return
        self.get_logger().info(
            f"call initial_pose time: {self._current_time.sec}.{self._current_time.nanosec}",
        )
        self._initial_pose_running = True
        self._initial_pose.header.stamp = self._current_time
        if self._initial_pose_method == InitializeLocalization.Request.AUTO:
            future_map_fit = self._map_fit_client.call_async(
                PoseWithCovarianceStampedSrv.Request(pose_with_covariance=self._initial_pose),
            )
            future_map_fit.add_done_callback(self.map_fit_cb)
        else:
            future_direct_init_pose = self._initial_pose_client.call_async(
                InitializeLocalization.Request(
                    method=self._initial_pose_method,
                    pose_with_covariance=[self._initial_pose],
                ),
            )
            future_direct_init_pose.add_done_callback(self.initial_pose_cb)

    def map_fit_cb(self, future: Future) -> None:
        result = future.result()
        if result is not None:
            if result.success:
                future_init_pose = self._initial_pose_client.call_async(
                    InitializeLocalization.Request(
                        method=self._initial_pose_method,
                        pose_with_covariance=[result.pose_with_covariance],
                    ),
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

    def initial_pose_cb(self, future: Future) -> None:
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

    def lookup_transform(
        self,
        stamp: Stamp,
        from_: str = "map",
        to: str = "base_link",
    ) -> TransformStamped:
        try:
            return self._tf_buffer.lookup_transform(
                from_,
                to,
                stamp,
                Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            return TransformStamped()

    def save_pkl(self, save_object: Any) -> None:
        PickleWriter(self._result_archive_path.joinpath("scene_result.pkl").as_posix(), save_object)

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

    def set_initial_pose(self) -> tuple[PoseWithCovarianceStamped | None, int | None]:
        # debug print(self._scenario.__dict__)
        dataset = self._scenario.Evaluation.Datasets[self._dataset_index]
        auto_pose = dataset.get("InitialPose")
        direct_pose = dataset.get("DirectInitialPose")
        if auto_pose is None and direct_pose is None:
            return None, None
        if auto_pose is not None:
            initial_pose: InitialPose = InitialPose(**auto_pose)
            pose_method: int = InitializeLocalization.Request.AUTO
        if direct_pose is not None:
            initial_pose: InitialPose = InitialPose(**direct_pose)
            pose_method: int = InitializeLocalization.Request.DIRECT
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
        return PoseWithCovarianceStamped(header=Header(frame_id="map"), pose=pose), pose_method

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
        index: int = array_classification.index(
            max(array_classification, key=lambda x: x.probability),
        )
        return array_classification[index]


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
