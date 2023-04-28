// Copyright 2022 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <driving_log_replayer/obstacle_segmentation_evaluator_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace driving_log_replayer
{
ObstacleSegmentationEvaluatorComponent::ObstacleSegmentationEvaluatorComponent(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("obstacle_segmentation_evaluator_component", options),
  buffer_(get_clock()),
  tf_listener_(buffer_),
  topic_rate_(false)
{
  std::string scenario_path;
  this->declare_parameter("scenario_path", "");
  this->get_parameter("scenario_path", scenario_path);

  try {
    scenario_yaml_ = YAML::LoadFile(scenario_path);
    if (scenario_yaml_["Evaluation"]["Conditions"]["NonDetection"].IsNull()) {
      proposed_area_ = {{}, 0.0, 0.0};
    } else {
      proposed_area_ =
        getProposedArea(scenario_yaml_["Evaluation"]["Conditions"]["NonDetection"]["ProposedArea"]);
    }
  } catch (YAML::Exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what());
    std::exit(EXIT_FAILURE);
  }

  input_pub_ = this->create_publisher<driving_log_replayer_msgs::msg::ObstacleSegmentationInput>(
    "obstacle_segmentation/input", 1);
  sub_lanelet_bin_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(
      &ObstacleSegmentationEvaluatorComponent::onLaneletMapBin, this, std::placeholders::_1));
  diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics_agg", 1,
    std::bind(&ObstacleSegmentationEvaluatorComponent::diagCallback, this, std::placeholders::_1));
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/obstacle_segmentation/pointcloud", rclcpp::QoS(1).best_effort(),
    std::bind(
      &ObstacleSegmentationEvaluatorComponent::pointsCallback, this, std::placeholders::_1));
}

void ObstacleSegmentationEvaluatorComponent::onLaneletMapBin(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

void ObstacleSegmentationEvaluatorComponent::diagCallback(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  for (const auto & diagnostic_status : msg->status) {
    if (
      diagnostic_status.name ==
      "/autoware/sensing/node_alive_monitoring/topic_status/ad_service_state_monitor: "
      "sensing_topic_status") {
      if (diagnostic_status.level >= diagnostic_status.ERROR) {
        topic_rate_ = false;
      } else {
        topic_rate_ = true;
      }
    }
  }
}

void ObstacleSegmentationEvaluatorComponent::pointsCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  // Rviz supports geometry_msgs::msg::Polygon by default, but not arrays.
  // MarkerArray is supported in Rviz, so display polygon array with line_strip array
  // The difference between polygon and line_strip is whether start and end are connected.
  visualization_msgs::msg::MarkerArray marker_bbox;

  geometry_msgs::msg::TransformStamped map_to_baselink;
  geometry_msgs::msg::TransformStamped baselink_to_map;
  try {
    map_to_baselink = buffer_.lookupTransform(
      "map", "base_link", msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
    baselink_to_map = buffer_.lookupTransform(
      "base_link", "map", msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    return;
  }

  // create MarkerArray(LineStrip) for proposed areas
  int current_intersection_count = 0;
  auto & [proposed_area, z_min, z_max] = proposed_area_;
  if (!proposed_area.empty()) {
    visualization_msgs::msg::Marker line_strip;
    // Perform a coordinate transformation and get the intersections with lanelet2.
    // convert points' coordinate from base_link to map
    auto [proposed_area_map, average_z] =
      transformToMap(proposed_area, msg->header, map_to_baselink);
    // get intersection of proposed area and lanelets
    auto intersection_lanelets =
      getIntersectionPolygon(proposed_area_map, baselink_to_map, z_min, z_max, average_z);
    for (const auto & intersection_lanelet : intersection_lanelets) {
      line_strip =
        getLineStripFromPointStampedArray(intersection_lanelet, ++current_intersection_count);
      marker_array.markers.emplace_back(line_strip);
    }
  }

  driving_log_replayer_msgs::msg::ObstacleSegmentationInput input_data;
  input_data.pointcloud = *msg;
  input_data.marker_array = marker_array;
  input_data.map_to_baselink = map_to_baselink;
  input_data.topic_rate = topic_rate_;
  input_pub_->publish(input_data);
}

visualization_msgs::msg::Marker
ObstacleSegmentationEvaluatorComponent::getLineStripFromPointStampedArray(
  std::vector<geometry_msgs::msg::PointStamped> point_array, int id)
{
  visualization_msgs::msg::Marker line_strip;
  line_strip.type = line_strip.LINE_STRIP;
  line_strip.action = line_strip.ADD;

  line_strip.ns = "intersection";
  line_strip.id = id;
  line_strip.color.r = 1.0;
  line_strip.color.g = 0.0;
  line_strip.color.b = 0.0;
  line_strip.color.a = 0.3;
  line_strip.scale.x = 0.2;
  line_strip.scale.y = 0.2;
  line_strip.scale.z = 0.2;
  line_strip.lifetime = rclcpp::Duration::from_seconds(0.2);

  for (const auto & point_stamped : point_array) {
    line_strip.header = point_stamped.header;
    line_strip.points.push_back(point_stamped.point);
  }
  return line_strip;
}

std::tuple<std::vector<geometry_msgs::msg::PointStamped>, double, double>
ObstacleSegmentationEvaluatorComponent::getProposedArea(const YAML::Node & proposed_area)
{
  std::vector<geometry_msgs::msg::PointStamped> point_array;
  std::vector<std::pair<double, double>> yaml_poly;

  yaml_poly = proposed_area["polygon_2d"].as<std::vector<std::pair<double, double>>>();
  double check_clock_wise = 0;

  for (unsigned int i = 0; i < yaml_poly.size(); i++) {
    auto p1 = yaml_poly[i];
    auto p2 = yaml_poly[(i + 1) % yaml_poly.size()];  // 最後のループで次の点は0に戻す
    check_clock_wise += (p2.first - p1.first) * (p2.second + p1.second);
  }

  if (check_clock_wise <= 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Input polygon is not clockwise");
    std::exit(EXIT_FAILURE);
  }

  for (auto & point : yaml_poly) {
    geometry_msgs::msg::PointStamped point_stamped;
    // Points in the array are stored in clockwise order.
    point_stamped.point.x = point.first;
    point_stamped.point.y = point.second;
    point_stamped.point.z = 0.0;
    point_array.emplace_back(point_stamped);
  }
  double z_min = proposed_area["z_min"].as<double>();
  double z_max = proposed_area["z_max"].as<double>();
  return {point_array, z_min, z_max};
}

std::tuple<std::vector<geometry_msgs::msg::PointStamped>, double>
ObstacleSegmentationEvaluatorComponent::transformToMap(
  std::vector<geometry_msgs::msg::PointStamped> point_stamped_array, std_msgs::msg::Header header,
  geometry_msgs::msg::TransformStamped transform)
{
  int point_count = 0;
  double total_z = 0.0;
  std::vector<geometry_msgs::msg::PointStamped> point_stamped_array_in_map;
  for (auto & point_stamped : point_stamped_array) {
    point_count++;
    point_stamped.header = header;
    geometry_msgs::msg::PointStamped point_stamped_in_map;
    tf2::doTransform(point_stamped, point_stamped_in_map, transform);
    total_z += point_stamped_in_map.point.z;
    point_stamped_array_in_map.emplace_back(point_stamped_in_map);
  }
  return {point_stamped_array_in_map, total_z / point_count};
}

std::vector<std::vector<geometry_msgs::msg::PointStamped>>
ObstacleSegmentationEvaluatorComponent::getIntersectionPolygon(
  const std::vector<geometry_msgs::msg::PointStamped> & point_stamped_poly,
  const geometry_msgs::msg::TransformStamped & baselink_to_map, const double & z_min,
  const double & z_max, const double & average_z)
{
  std::vector<std::vector<geometry_msgs::msg::PointStamped>> out;  // final result
  Polygon2d poly = toBoostPoly(point_stamped_poly);

  // get the polygon overlapping the lane of the road
  for (const auto & road_lanelet : road_lanelets_) {
    std::vector<Polygon2d> intersections;
    boost::geometry::intersection(
      poly, toBoostPoly(road_lanelet.polygon2d().basicPolygon()), intersections);
    for (const auto & intersection : intersections) {
      out.emplace_back(setGeometryPointArray(
        intersection, point_stamped_poly[0].header, z_min, z_max, baselink_to_map, average_z));
    }
  }
  return out;
}

Polygon2d ObstacleSegmentationEvaluatorComponent::toBoostPoly(
  const std::vector<geometry_msgs::msg::PointStamped> & polygon)
{
  Polygon2d boost_poly;
  for (const auto & point_stamped : polygon) {
    const Point2d point2d(point_stamped.point.x, point_stamped.point.y);
    boost_poly.outer().push_back(point2d);
  }
  return boost_poly;
}

Polygon2d ObstacleSegmentationEvaluatorComponent::toBoostPoly(
  const lanelet::BasicPolygon2d & polygon)
{
  Polygon2d boost_poly;
  for (const auto & vec : polygon) {
    const Point2d point2d(vec.x(), vec.y());
    boost_poly.outer().push_back(point2d);
  }

  return boost_poly;
}

std::vector<geometry_msgs::msg::PointStamped>
ObstacleSegmentationEvaluatorComponent::setGeometryPointArray(
  const Polygon2d & polygon, const std_msgs::msg::Header & header, const double & z_min,
  const double & z_max, const geometry_msgs::msg::TransformStamped & baselink_to_map,
  const double & average_z)
{
  std::vector<geometry_msgs::msg::PointStamped> vector_poly_without_z;
  std::vector<geometry_msgs::msg::PointStamped> vector_poly;
  for (const auto & point2d : polygon.outer()) {
    geometry_msgs::msg::PointStamped point_stamped_in_map;
    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped_in_map.header = header;
    point_stamped_in_map.point.x = point2d.x();
    point_stamped_in_map.point.y = point2d.y();
    point_stamped_in_map.point.z = average_z;
    tf2::doTransform(point_stamped_in_map, point_stamped, baselink_to_map);
    vector_poly_without_z.emplace_back(point_stamped);
  }
  // set z lower layer in base_link
  for (auto & point : vector_poly_without_z) {
    point.point.z = z_min;
    vector_poly.emplace_back(point);
  }
  // set z upper layer in base_link
  for (auto & point : vector_poly_without_z) {
    point.point.z = z_max;
    vector_poly.emplace_back(point);
  }
  return vector_poly;
}

}  // namespace driving_log_replayer

RCLCPP_COMPONENTS_REGISTER_NODE(driving_log_replayer::ObstacleSegmentationEvaluatorComponent)
