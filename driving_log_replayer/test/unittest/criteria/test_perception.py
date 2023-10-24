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

import pytest

from driving_log_replayer.criteria.perception import CriteriaLevel


def test_criteria_level_from_str_ok() -> None:
    criteria_level = CriteriaLevel.from_str("perfect")
    assert criteria_level == CriteriaLevel.PERFECT


def test_criteria_level_from_str_ng() -> None:
    with pytest.raises(AssertionError):
        CriteriaLevel.from_str("not_criteria_level_type")


def test_criteria_level_from_number_ok() -> None:
    allowed_value = 50.0
    criteria_level = CriteriaLevel.from_number(allowed_value)
    assert criteria_level == CriteriaLevel.CUSTOM
    assert criteria_level.value == allowed_value


def test_criteria_level_from_number_ng() -> None:
    with pytest.raises(AssertionError):
        CriteriaLevel.from_number(110.0)
