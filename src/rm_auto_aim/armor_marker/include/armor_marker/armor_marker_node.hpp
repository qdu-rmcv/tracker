#ifndef ARMOR_MARKER__ARMOR_MARKER_NODE_HPP_
#define ARMOR_MARKER__ARMOR_MARKER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <auto_aim_interfaces/msg/trajectory_info.hpp>

#include "planning_trajectory/trajectory_solver.hpp"

namespace rm_auto_aim
{
class ArmorMarkerNode : public rclcpp::Node
{
public:
  explicit ArmorMarkerNode(const rclcpp::NodeOptions& options);

private:
  /// 将 Target msg 转换为 TrajectorySolver::Target 并调用 PredictCenter/PredictArmor
  void TrackerTargetCallback(
      const auto_aim_interfaces::msg::Target::SharedPtr target_msg);

  void DetectorArmorsCallback(
      const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg);

  void TrajectoryInfoCallback(
      const auto_aim_interfaces::msg::TrajectoryInfo::SharedPtr info_msg);

  // TrajectorySolver 用于整车建模预测装甲板位置
  std::unique_ptr<TrajectorySolver> solver_;

  // Detector marker 相关
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detector_marker_pub_;
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;

  // Tracker marker 相关
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracker_marker_pub_;
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker tracker_armor_marker_;

  // Trajectory info marker 相关
  rclcpp::Subscription<auto_aim_interfaces::msg::TrajectoryInfo>::SharedPtr
      trajectory_info_sub_;
  visualization_msgs::msg::Marker aiming_point_marker_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_MARKER__ARMOR_MARKER_NODE_HPP_
