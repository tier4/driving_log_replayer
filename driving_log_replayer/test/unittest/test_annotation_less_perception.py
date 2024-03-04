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

from pathlib import Path

from driving_log_replayer.annotation_lees_perception import AnnotationLessPerceptionScenario
from driving_log_replayer.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: AnnotationLessPerceptionScenario = load_sample_scenario(
        "annotation_less_perception",
        AnnotationLessPerceptionScenario,
    )
    assert scenario.ScenarioName == "sample_annotation_less_perception"


def test_path_with_empty_str() -> None:
    f = Path("")  # noqa
    assert f.exists()
