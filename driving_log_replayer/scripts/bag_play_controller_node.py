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

import sys
from typing import Callable

import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rosbag2_interfaces.srv import IsPaused
from rosbag2_interfaces.srv import Pause
from rosbag2_interfaces.srv import Resume


class BagPlayController(Node):
    def __init__(self) -> None:
        super().__init__("bag_play_controller")
        self.declare_parameter("pause_sec", 15)
        self.declare_parameter("shutdown_sec", 5)
        self._pause_state = True
        self._pause_sec = self.get_parameter("pause_sec").get_parameter_value().integer_value
        self._shutdown_sec = self.get_parameter("shutdown_sec").get_parameter_value().integer_value
        self._pause_client = self.create_client(Pause, "/rosbag2_player/pause")
        self._resume_client = self.create_client(Resume, "/rosbag2_player/resume")
        self._is_paused_client = self.create_client(IsPaused, "/rosbag2_player/is_paused")
        self._pause_counter = 0
        self._shutdown_counter = 0
        self._current_time = Time().to_msg()
        self._prev_time = Time().to_msg()
        self._timer = self.create_timer(
            1,
            self._timer_cb,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer

    def print_state(self, future: Future) -> None:
        res = future.result()
        if res is not None:
            self._pause_state = res.paused
            self.get_logger().info(f"IsPaused: {self._pause_state}")
        else:
            self.get_logger().error(f"Exception while calling service: {future.exception()}")

    def trigger_result_cb(self, name: str) -> Callable:
        def impl(future: Future) -> None:
            res = future.result()
            if res is not None:
                self.get_logger().info(f"Response received for: {name}")
            else:
                self.get_logger().error(f"Exception for service: {future.exception()}")

        return impl

    def timer_cb(self) -> None:
        self._prev_time = self._current_time
        self._current_time = self.get_clock().now().to_msg()
        req = IsPaused.Request()
        future = self._is_paused_client.call_async(req)
        future.add_done_callback(self._print_state)
        if self._current_time.sec > 0:
            if self._pause_counter == 0:
                # stop bag play
                self.pause()
            elif self._pause_counter == self._pause_sec:
                self.resume()
            self._pause_counter += 1
            if not self._pause_state and (self._prev_time == self._current_time):
                self._shutdown_counter += 1
            else:
                self._shutdown_counter = 0
            if self._shutdown_counter > self._shutdown_sec:
                sys.exit(1)

    def pause(self) -> None:
        req = Pause.Request()
        future = self._pause_client.call_async(req)
        future.add_done_callback(self._trigger_result_cb("Pause"))

    def resume(self) -> None:
        req = Resume.Request()
        future = self._resume_client.call_async(req)
        future.add_done_callback(self._trigger_result_cb("Resume"))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    bag_play_controller = BagPlayController()
    rclpy.spin(bag_play_controller)
    bag_play_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
