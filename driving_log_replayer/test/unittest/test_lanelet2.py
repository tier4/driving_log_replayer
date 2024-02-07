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

from typing import Any

import lanelet2
from lanelet2.core import getId
from lanelet2.core import Lanelet
from lanelet2.core import LineString3d
from lanelet2.core import Point3d
from lanelet2_extension_python.projection import MGRSProjector
from lanelet2_extension_python.utility import utilities
import lanelet2_extension_python.utility.query as query
from shapely import intersection
from shapely import Polygon

# refer lanelet2_example
# https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/scripts/tutorial.py


def get_linestring_at_y(y: float) -> LineString3d:
    return LineString3d(getId(), [Point3d(getId(), i, y, 0) for i in range(3)])


def get_a_lanelet(index: float = 0) -> Lanelet:
    return Lanelet(getId(), get_linestring_at_y(2 + index), get_linestring_at_y(0 + index))


def convert_lanelet_to_shapely_polygon(lanelet: Lanelet) -> Polygon:
    points: list[float, float] = []
    for p_2d in lanelet.polygon2d():
        points.append([p_2d.x, p_2d.y])
    return Polygon(points)


def get_road_lanelet_from_sample_map() -> Any:
    map_path = "/home/hyt/map/oss/lanelet2_map.osm"
    projection = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
    lanelet_map = lanelet2.io.load(map_path, projection)
    all_lanelets = query.laneletLayer(lanelet_map)
    road_lanelets = query.roadLanelets(all_lanelets)
    for road in road_lanelets:
        p_2d = road.polygon2d()
        line_string = utilities.lineStringToPolygon(road.leftBound)
        for point in p_2d:
            print(point)
        # for l_p in line_string:
        #     print(l_p)


def test_intersection() -> None:
    lanelet = get_a_lanelet()
    l_poly = convert_lanelet_to_shapely_polygon(lanelet)
    s_poly = Polygon(((1.5, 3.0), (4.0, 3.0), (4.0, -1.0), (1.5, -1.0)))
    i_area = intersection(l_poly, s_poly)
    assert i_area == Polygon(((2, 2), (2, 0), (1.5, 0), (1.5, 2)))


def test_intersection_no_overlapping() -> None:
    lanelet = get_a_lanelet()
    l_poly = convert_lanelet_to_shapely_polygon(lanelet)
    s_poly = Polygon(((-1.0, -1.0), (-1.0, -2.0), (-2.0, -2.0), (-2.0, -1.0)))
    i_area = intersection(l_poly, s_poly)
    assert i_area.is_empty
