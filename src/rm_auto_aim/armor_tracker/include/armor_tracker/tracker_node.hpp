#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <string>

#include "armor_tracker/tracker.hpp"
#include "armor_tracker/tracker_params.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace rm_auto_aim
{

using armors_tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;

class ArmorTrackerNode : public rclcpp::Node
{
 public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions& options);

 private:
  // 读取所有参数到 TrackerParams 结构体
  TrackerParams DeclareTrackerParams();

  // 订阅回调
  void ArmorsCallback(auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  // -------------------- 配置 --------------------
  double max_armor_distance_ = 10.0;
  bool is_hero_ = false;
  std::string target_frame_ = "odom";
  std::string last_camera_frame_id_ = "camera_optical_frame";

  // 时间相关（用于把 lost/change 的秒数换算成帧数）
  rclcpp::Time last_time_;
  double dt_ = 0.01;
  double lost_time_thres_ = 0.3;
  double change_time_thres_ = 0.3;

  // -------------------- 核心 --------------------
  std::unique_ptr<Tracker> tracker_;

  // -------------------- TF / 订阅 --------------------
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<armors_tf2_filter> armors_filter_;

  // -------------------- 发布 --------------------
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
