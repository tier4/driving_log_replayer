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


from perception_eval.evaluation import PerceptionFrameResult

from driving_log_replayer.traffic_light import Perception


def test_perception_success() -> None:
    evaluation_item = Perception(condition={"PassRate": 99.0})
    evaluation_item.set_frame(PerceptionFrameResult())


def test_perception_fail() -> None:
    evaluation_item = Perception(condition={"PassRate": 99.0})
