#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

#include <image_transport/publisher.hpp>
#include <memory>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "armor_detector/armor.hpp"
#include "armor_detector/armor_pose_optimizer.hpp"
#include "armor_detector/detector_factory.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{

class ArmorDetectorNode : public rclcpp::Node
{
 public:
  ArmorDetectorNode(const rclcpp::NodeOptions& options);

 private:
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& IMG_MSG);

  std::unique_ptr<PnPSolver> InitPnPSolver();
  std::unique_ptr<ArmorPoseOptimizer> InitPoseOptimizer();
  void InitTransformListener();

  std::vector<Armor> DetectArmors(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

  void CreateDebugPublishers();
  void DestroyDebugPublishers();

  // Armor Detector
  std::unique_ptr<DetectorBase> detector_;
  std::unique_ptr<PnPSolver> pnp_solver_;
  std::unique_ptr<ArmorPoseOptimizer> pose_optimizer_;

  // tf2
  std::string odom_frame_;
  Eigen::Matrix3d gimbal_to_camera_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::string current_frame_id_;

  // Camera switch
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_switch_sub_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;

  std::vector<std::pair<std::string, uint64_t>> debug_latencies_;  // 各环节延迟
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
