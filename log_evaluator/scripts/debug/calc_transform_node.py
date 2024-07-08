#!/usr/bin/env python3

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

import json
from math import pi

from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.time import Time
from rosidl_runtime_py import message_to_ordereddict
from std_msgs.msg import Header
from tf2_ros import Buffer
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros import TransformListener
from tf_transformations import quaternion_from_euler


class CalcTransformNode(Node):
    def __init__(self) -> None:
        super().__init__("calc_transform_node")

        self._current_time = Time().to_msg()
        self._prev_time = Time().to_msg()

        self._tf_broadcaster = TransformBroadcaster(self)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._translation = Vector3(x=10.0, y=10.0, z=0.0)
        q = quaternion_from_euler(0.0, 0.0, -pi / 2)
        self.get_logger().error(f"{q[0]=}, {q[1]=}, {q[1]=}, {q[2]=}")
        self._rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self._timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer

    def timer_cb(self) -> None:
        # send transform and lookup transform
        self._current_time = self.get_clock().now().to_msg()

        t = TransformStamped(
            header=Header(stamp=self._current_time, frame_id="map"),
            child_frame_id="base_link",
            transform=Transform(
                translation=self._translation,
                rotation=self._rotation,
            ),
        )
        self._tf_broadcaster.sendTransform(t)
        base_link_to_map = self.lookup_transform(self._prev_time, "base_link", "map")
        txt_base_link_to_map = json.dumps(message_to_ordereddict(base_link_to_map))
        self.get_logger().error(txt_base_link_to_map)
        self._prev_time = self._current_time

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


def main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    evaluator = CalcTransformNode()
    executor.add_node(evaluator)
    executor.spin()
    evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
