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

import simplejson as json

from driving_log_replayer.annotation_lees_perception import AnnotationLessPerceptionScenario
from driving_log_replayer.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: AnnotationLessPerceptionScenario = load_sample_scenario(
        "annotation_less_perception",
        AnnotationLessPerceptionScenario,
    )
    assert scenario.ScenarioName == "sample_annotation_less_perception"


annotation_less_threshold = '{"lateral_deviation": {"min": 0.0007497261146496814, "max": 0.01901246019108281, "mean": 0.0067277356687898025}, "yaw_deviation": {"min": 0.10563033917197448, "max": 2.690667243630572, "mean": 1.1227060414012764}, "predicted_path_deviation_5.00": {"min": 0.0, "max": 4.808206641719744, "mean": 0.7326834458598731}, "predicted_path_deviation_3.00": {"min": 0.0, "max": 2.5071612324840795, "mean": 0.3996540127388532}, "predicted_path_deviation_2.00": {"min": 0.0, "max": 1.5049176799363064, "mean": 0.2566065429936305}, "predicted_path_deviation_1.00": {"min": 0.0, "max": 0.6819488455414007, "mean": 0.1296180700636943}}'
j_threshold = json.loads(annotation_less_threshold)
print(j_threshold)
