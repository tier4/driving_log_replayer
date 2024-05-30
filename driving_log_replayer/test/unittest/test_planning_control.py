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

# from diagnostic_msgs.msg import KeyValue

# from driving_log_replayer.planning_control import LowerUpper
# from driving_log_replayer.planning_control import ValuesAfter0

# kv = KeyValue(key="pos_x", value=1.0)
# key = kv.key
# detail_cond = ValuesAfter0(
#     pos_x=LowerUpper(lower=0.5, upper=2.0),
#     pos_y=LowerUpper(lower=0.5, upper=2.0),
# )
# print(detail_cond.key)

from dataclasses import dataclass


@dataclass
class PosVel:
    pos_x: float = 1.0
    pos_y: float = 2.0
    vel: float = 3.0


pos_vel = PosVel()
key_var = "pos_x"

print(getattr(pos_vel, key_var))
