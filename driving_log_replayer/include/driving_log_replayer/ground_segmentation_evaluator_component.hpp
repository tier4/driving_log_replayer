#pragma once

#include "driving_log_replayer/visibility_control.hpp"

#include <autoware_point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

namespace driving_log_replayer
{
using autoware_point_types::PointXYZIE;
using sensor_msgs::msg::PointCloud2;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
using Sync = message_filters::TimeSynchronizer<PointCloud2, PointCloud2>;
class GroundSegmentationEvaluatorComponent : public rclcpp::Node
{
public:
  DRIVING_LOG_REPLAYER_PUBLIC
  explicit GroundSegmentationEvaluatorComponent(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<Sync> sync_ptr_;
  message_filters::Subscriber<PointCloud2> concat_cloud_sub_;
  message_filters::Subscriber<PointCloud2> non_ground_cloud_sub_;
  void evaluate(
    const PointCloud2::ConstSharedPtr ground_truth_cloud,
    const PointCloud2::ConstSharedPtr eval_target_cloud);

  std::tuple<float, float, float, float> compute_metrics(
    const float TP, const float FP, const float TN, const float FN);
};
}  // namespace driving_log_replayer
