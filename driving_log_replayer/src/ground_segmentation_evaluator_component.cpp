#include "../include/driving_log_replayer/ground_segmentation_evaluator_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace driving_log_replayer
{
GroundSegmentationEvaluatorComponent::GroundSegmentationEvaluatorComponent(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("ground_segmentation_evaluator_component", options)
{
  eval_result_pub_ =
    this->create_publisher<driving_log_replayer_msgs::msg::GroundSegmentationEvalResult>(
      "ground_segmentation/evaluation/result", rclcpp::QoS(10).best_effort());
  dbg_invalid_fp_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("dbg_invalid_fp_cloud", rclcpp::QoS(1));
  concat_cloud_sub_.subscribe(
    this, "/sensing/lidar/concatenated/pointcloud", rmw_qos_profile_sensor_data);
  non_ground_cloud_sub_.subscribe(
    this, "/perception/obstacle_segmentation/single_frame/pointcloud_raw",
    rmw_qos_profile_sensor_data);
  sync_ptr_ = std::make_shared<Sync>(concat_cloud_sub_, non_ground_cloud_sub_, 1000);
  sync_ptr_->registerCallback(std::bind(
    &GroundSegmentationEvaluatorComponent::evaluate, this, std::placeholders::_1,
    std::placeholders::_2));
}

void GroundSegmentationEvaluatorComponent::evaluate(
  const PointCloud2::ConstSharedPtr ground_truth_cloud,
  const PointCloud2::ConstSharedPtr eval_target_cloud)
{
  rclcpp::Time gt_cloud_ts = rclcpp::Time(ground_truth_cloud->header.stamp);
  rclcpp::Time eval_target_cloud_ts = rclcpp::Time(eval_target_cloud->header.stamp);

  if (
    std::find_if(
      eval_target_cloud->fields.begin(), eval_target_cloud->fields.end(),
      [](sensor_msgs::msg::PointField f) { return f.name == "entity_id"; }) ==
    eval_target_cloud->fields.end()) {
    RCLCPP_WARN(this->get_logger(), "input cloud doesn't have \"entity_id\" field !!");
    return;
  };
  sensor_msgs::PointCloud2ConstIterator<int32_t> gt_label_itr(*ground_truth_cloud, "entity_id");
  sensor_msgs::PointCloud2ConstIterator<int32_t> target_label_itr(*eval_target_cloud, "entity_id");

  // tp : 地面として認識されてる地面点群
  // tn : 障害物として認識されてる障害物点群
  // fp : 地面として認識されてる障害物点群
  // fn : 障害物として認識されてる地面点群

  // ground label(positive)の総数(tp+fn)とnon ground label(negative)の総数(tn+fp)を取得
  std::size_t true_ground_cnt = 0;
  std::size_t true_non_ground_cnt = 0;
  for (; gt_label_itr != gt_label_itr.end(); ++gt_label_itr) {
    if (*gt_label_itr == 1) {
      ++true_ground_cnt;
    } else {
      ++true_non_ground_cnt;
    }
  }

  // segmentation後の点群におけるground label(false negative)とnon ground label(true
  // negative)を取得
  std::size_t fn = 0;
  std::size_t tn = 0;
  for (; target_label_itr != target_label_itr.end(); ++target_label_itr) {
    if (*target_label_itr == 1) {
      ++fn;
    } else {
      ++tn;
    }
  }

  // tpとfp計算
  std::size_t tp = true_ground_cnt - fn;
  std::size_t fp = true_non_ground_cnt - tn;

  auto [precision, recall, specificity, f1_score] = compute_metrics(tp, fp, tn, fn);
  // RCLCPP_INFO(this->get_logger(), "==============================");
  // RCLCPP_INFO_STREAM(
  //   this->get_logger(),
  //   "eval target cloud sum ? : " << std::boolalpha << ((tn + fp) == ));
  // RCLCPP_INFO(this->get_logger(), "true ground point : %ld", true_ground_cnt);
  // RCLCPP_INFO(this->get_logger(), "true non ground point : %ld", true_non_ground_cnt);
  // RCLCPP_INFO(this->get_logger(), "before cloud ts : %lf", gt_cloud_ts.seconds());
  // RCLCPP_INFO(this->get_logger(), "after cloud ts : %lf", eval_target_cloud_ts.seconds());
  // RCLCPP_INFO(this->get_logger(), "TP : %ld, FP : %ld, TN : %ld, FN : %ld", tp, fp, tn, fn);
  // RCLCPP_INFO(
  //   this->get_logger(), "Precision : %f, Recall : %f, Specificity : %f, F1 : %f", precision,
  //   recall, specificity, f1_score);
  // RCLCPP_INFO(this->get_logger(), "==============================");

  driving_log_replayer_msgs::msg::GroundSegmentationEvalResult eval_result_msg;
  eval_result_msg.tp = tp;
  eval_result_msg.fp = fp;
  eval_result_msg.tn = tn;
  eval_result_msg.fn = fn;
  eval_result_msg.precision = precision;
  eval_result_msg.recall = recall;
  eval_result_msg.specificity = specificity;
  eval_result_msg.f1_score = f1_score;

  eval_result_pub_->publish(eval_result_msg);
}

std::tuple<float, float, float, float> GroundSegmentationEvaluatorComponent::compute_metrics(
  const float TP, const float FP, const float TN, const float FN)
{
  float precision = static_cast<float>(TP) / (TP + FP);
  float recall = static_cast<float>(TP) / (TP + FN);
  float specificity = static_cast<float>(TN) / (TN + FP);
  float f1_score = 2 * (precision * recall) / (precision + recall);
  return {precision, recall, specificity, f1_score};
}
}  // namespace driving_log_replayer

RCLCPP_COMPONENTS_REGISTER_NODE(driving_log_replayer::GroundSegmentationEvaluatorComponent)