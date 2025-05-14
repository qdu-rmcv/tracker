// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <auto_aim_interfaces/msg/detail/all_latency__struct.hpp>
#include <auto_aim_interfaces/msg/detail/receive__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_interfaces/msg/receive.hpp"
#include "armor_executor/SolveTrajectory.hpp"

#include "auto_aim_interfaces/msg/all_latency.hpp"

#include "std_msgs/msg/float64.hpp"
#include "auto_aim_interfaces/msg/test.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
using velocity_tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Velocity>;
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  void velocityCallback(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg);
  void receiveCallback(const auto_aim_interfaces::msg::Receive::SharedPtr receive_msg);
  void latencyCallback(const auto_aim_interfaces::msg::AllLatency::SharedPtr latency_msg);

  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;

  

  // Armor tracker
  double s2qxyz_max_, s2qxyz_min_, s2qyaw_max_, s2qyaw_min_, s2qr_;
  double r_xyz_factor, r_yaw;
  double lost_time_thres_;
  float send_pitch=0, send_yaw=0, aim_x=0, aim_y=0, aim_z=0;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<SolveTrajectory> gaf_solver;

  // 延迟
  rclcpp::Publisher<auto_aim_interfaces::msg::AllLatency>::SharedPtr tracker_latency_pub_;

  // 测试
  rclcpp::Publisher<auto_aim_interfaces::msg::Test>::SharedPtr test_pub_;
  float target0_yaw;

  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  // Change target service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_target_srv_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Receive>::SharedPtr receive_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::AllLatency>::SharedPtr detctor_sub_;
  std::shared_ptr<velocity_tf2_filter> velocity_filter_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker trackered_armor_marker_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 测试加加速度恒定用
  double s2qxyz_;
  double s2qyaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
