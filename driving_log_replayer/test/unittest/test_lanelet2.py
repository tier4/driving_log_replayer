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

from geometry_msgs.msg import Point
import lanelet2  # noqa
from lanelet2.core import getId
from lanelet2.core import Lanelet
from lanelet2.core import LineString3d
from lanelet2.core import Point3d
from lanelet2_extension_python.utility.query import getLaneletsWithinRange
from shapely.geometry import Polygon

from driving_log_replayer.lanelet2_util import to_shapely_polygon

# refer lanelet2_example
# https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/scripts/tutorial.py


def get_linestring_at_y(y: float) -> LineString3d:
    return LineString3d(getId(), [Point3d(getId(), i, y, 0) for i in range(3)])


def get_a_lanelet(index: float = 0) -> Lanelet:
    return Lanelet(getId(), get_linestring_at_y(2 + index), get_linestring_at_y(0 + index))


def test_intersection() -> None:
    lanelet = get_a_lanelet()
    l_poly = to_shapely_polygon(lanelet)
    s_poly = Polygon(((1.5, 3.0), (4.0, 3.0), (4.0, -1.0), (1.5, -1.0)))
    i_area = l_poly.intersection(s_poly)
    assert i_area == Polygon(((2, 2), (2, 0), (1.5, 0), (1.5, 2)))


def test_intersection_no_overlapping() -> None:
    lanelet = get_a_lanelet()
    l_poly = to_shapely_polygon(lanelet)
    s_poly = Polygon(((-1.0, -1.0), (-1.0, -2.0), (-2.0, -2.0), (-2.0, -1.0)))
    i_area = l_poly.intersection(s_poly)
    assert i_area.is_empty


def test_get_lanelets_within_range() -> None:
    lanes = [get_a_lanelet(), get_a_lanelet(index=4)]
    near_lanelets = getLaneletsWithinRange(lanes, Point(x=1.0, y=3.0, z=0.0), 1.0)
    assert len(near_lanelets) == 2  # noqa


def test_get_lanelets_within_range_no_lane() -> None:
    lanes = [get_a_lanelet(), get_a_lanelet(index=4)]
    near_lanelets = getLaneletsWithinRange(lanes, Point(x=1.0, y=3.0, z=0.0), 0.5)
    assert len(near_lanelets) == 0
