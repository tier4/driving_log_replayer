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

import contextlib
import os
from threading import Thread
import time

import domain_coordinator
import launch
import launch_pytest
import launch_ros.actions
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

stack = contextlib.ExitStack()
if "ROS_DOMAIN_ID" not in os.environ and "DISABLE_ROS_ISOLATION" not in os.environ:
    domain_id = stack.enter_context(domain_coordinator.domain_id())
    print("ROS_DOMAIN_ID is", domain_id)  # noqa
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)


@launch_pytest.fixture()
def generate_test_description() -> launch_ros.actions.Node:
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="driving_log_replayer",
                executable="localization_evaluator_node.py",
            ),
        ],
    )


@pytest.mark.launch(fixture=generate_test_description)
def test_accumulation(make_test_node):
    node = make_test_node
    for i in range(10):
        node.publish(Int32(data=i))
        time.sleep(0.01)

    end_time = time.time() + 5
    while time.time() < end_time:
        if len(node.msgs) == 10:
            break
        time.sleep(0.1)

    assert len(node.msgs) == 10
    assert node.msgs[-1] == Int32(data=45)


@pytest.fixture
def make_test_node():
    rclpy.init()
    node = MakeTestNode()
    node.start()
    yield node
    node.destroy_node()
    rclpy.shutdown()


class MakeTestNode(Node):
    def __init__(self) -> None:
        super().__init__("test_node")

    def start(self):
        self._pub = self.create_publisher(Int32, "src", 10)
        self._sub = self.create_subscription(Int32, "dst", self._msg_received, 10)
        self.msgs = []
        self._ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node),
            args=(self,),
        )
        self._ros_spin_thread.start()

        self._wait_for_connect()

    def publish(self, data):
        self._pub.publish(data)

    def _msg_received(self, msg):
        self.msgs.append(msg)

    def _wait_for_connect(self, timeout_s=5):
        end_time = time.time() + timeout_s
        while time.time() < end_time:
            cnt = self._pub.get_subscription_count()
            if cnt > 0:
                return True
            time.sleep(0.1)
        return False
