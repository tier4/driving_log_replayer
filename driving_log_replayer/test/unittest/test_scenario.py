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

from pydantic import BaseModel


class SampleModel(BaseModel):
    version: str
    name: str


def test_type_check_base_model() -> None:
    sample_model = SampleModel(version="1.0.0", name="s_model")
    assert type(sample_model) is not BaseModel
    assert isinstance(sample_model, BaseModel)
