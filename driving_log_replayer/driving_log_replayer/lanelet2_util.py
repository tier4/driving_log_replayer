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

from typing import Any

import lanelet2
from lanelet2.core import Lanelet
from lanelet2_extension_python.projection import MGRSProjector
from lanelet2_extension_python.utility import query
from shapely.geometry import Polygon


def road_lanelets_from_file(map_path: str) -> Any:
    projection = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
    lanelet_map = lanelet2.io.load(map_path, projection)
    all_lanelets = query.laneletLayer(lanelet_map)
    # return type lanelet2_extension_python._lanelet2_extension_python_boost_python_utility.lanelet::ConstLanelets
    return query.roadLanelets(all_lanelets)


def to_shapely_polygon(lanelet: Lanelet) -> Polygon:
    points: list[float, float] = []
    for p_2d in lanelet.polygon2d():
        points.append([p_2d.x, p_2d.y])
    return Polygon(points)
