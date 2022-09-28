// Copyright 2015-2022 TIER IV, Inc. All rights reserved.
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

#ifndef DRIVING_LOG_REPLAYER__OBSTACLE_SEGMENTATION_EVALUATOR_COMPONENT_HPP_
#define DRIVING_LOG_REPLAYER__OBSTACLE_SEGMENTATION_EVALUATOR_COMPONENT_HPP_

#include "driving_log_replayer/visibility_control.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <driving_log_replayer_msgs/msg/obstacle_segmentation_input.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace driving_log_replayer
{
using Point2d = boost::geometry::model::d2::point_xy<double>;
using Polygon2d = boost::geometry::model::polygon<Point2d, true, false>;  // clockwise, open
class ObstacleSegmentationEvaluatorComponent : public rclcpp::Node
{
public:
  DRIVING_LOG_REPLAYER_PUBLIC
  explicit ObstacleSegmentationEvaluatorComponent(const rclcpp::NodeOptions & options);

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;
  YAML::Node scenario_yaml_;
  std::tuple<std::vector<geometry_msgs::msg::PointStamped>, double, double> proposed_area_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  void pointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  rclcpp::Publisher<driving_log_replayer_msgs::msg::ObstacleSegmentationInput>::SharedPtr
    input_pub_;
  visualization_msgs::msg::Marker getLineStripFromPointStampedArray(
    const std::vector<geometry_msgs::msg::PointStamped> array_point_stamped, int id);
  lanelet::LaneletMapPtr lanelet_map_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_lanelet_bin_map_;
  void onLaneletMapBin(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
  visualization_msgs::msg::MarkerArray getVisProposedAreas(const YAML::Node & proposed_areas);
  std::tuple<std::vector<geometry_msgs::msg::PointStamped>, double, double> getProposedArea(
    const YAML::Node & proposed_areas);
  lanelet::ConstLanelets road_lanelets_;
  std::tuple<std::vector<geometry_msgs::msg::PointStamped>, double> transformToMap(
    const std::vector<geometry_msgs::msg::PointStamped> point_stamped_array,
    const std_msgs::msg::Header header, geometry_msgs::msg::TransformStamped transform);
  std::vector<std::vector<geometry_msgs::msg::PointStamped>> getIntersectionPolygon(
    const std::vector<geometry_msgs::msg::PointStamped> & point_stamped_array,
    const geometry_msgs::msg::TransformStamped & baselink_to_map, const double & z_min,
    const double & z_max, const double & average_z);
  Polygon2d toBoostPoly(const std::vector<geometry_msgs::msg::PointStamped> & polygon);
  Polygon2d toBoostPoly(const lanelet::BasicPolygon2d & polygon);
  std::vector<geometry_msgs::msg::PointStamped> setGeometryPointArray(
    const Polygon2d & polygon, const std_msgs::msg::Header & header, const double & z_min,
    const double & z_max, const geometry_msgs::msg::TransformStamped & baselink_to_map,
    const double & average_z);
  bool topic_rate_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);
};
}  // namespace driving_log_replayer

#endif  // DRIVING_LOG_REPLAYER__OBSTACLE_SEGMENTATION_EVALUATOR_COMPONENT_HPP_
