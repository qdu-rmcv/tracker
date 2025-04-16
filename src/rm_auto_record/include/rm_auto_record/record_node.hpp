// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_AUTO_RECORD__RECORD_NODE_HPP_
#define RM_AUTO_RECORD__RECORD_NODE_HPP_

#include <rmw/qos_profiles.h>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

// Include the message header file here
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
// end of the inclusion

namespace rm_auto_record
{
class RecordNode : public rclcpp::Node
{
public:
  explicit RecordNode(const rclcpp::NodeOptions & options);

private:
  void write(std::shared_ptr<rclcpp::SerializedMessage> msg, int index);

  void recordInit(std_msgs::msg::String::SharedPtr msg);

  std::shared_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;

  std::string uri_;

  // Declare the message subscriber type here
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  // end of the declaration

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_controller_sub_;

  enum RecordState {
    WAIT,
    INIT,
    RECORDING,
  } record_state_;

  struct TopicInfo
  {
    std::string topic_name;
    std::string topic_type;
  };

  std::vector<TopicInfo> topic_info_;
};

}  // namespace rm_auto_record

#endif  // RM_AUTO_RECORD__RECORD_NODE_HPP_