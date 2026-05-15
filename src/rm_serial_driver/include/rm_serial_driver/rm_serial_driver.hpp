#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// STD
#include <chrono>
#include <memory>

// ROS2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

// LibXR
#include "SharedTopic.hpp"
#include "SharedTopicClient.hpp"
#include "linux_uart.hpp"

// ROS2自定义消息包
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"

namespace rm_serial_driver
{

/*LibXR相关*/

// LibXR应用程序入口函数
static void XRobotMain(LibXR::HardwareContainer& hw)
{
  using namespace LibXR;
  static ApplicationManager appmgr;

  static SharedTopic shared_topic(hw, appmgr, "uart_client", 256,
                                  {{"ahrs_quaternion"}, {"lob_shot"}});

  static SharedTopicClient shared_topic_client(
      hw, appmgr, "uart_client", 256,
      {{"target_euler"}, {"fire_notify", "tracker"}, {"target_num"}});
}

#pragma pack(push, 1)
struct HostEulerTarget
{
  float rol, pit, yaw;
  float rol_dot, pit_dot, yaw_dot;
  float rol_ddot, pit_ddot, yaw_ddot;
};
#pragma pack(pop)

class RMSerialDriver : public rclcpp::Node
{
 public:
  explicit RMSerialDriver(const rclcpp::NodeOptions& options);
  ~RMSerialDriver() override;

 private:
  uint8_t fire_notify_ = 1;
  double timestamp_offset_{};

  /* 函数声明 */

  // Send消息回调函数
  void SendCallBack(const auto_aim_interfaces::msg::Send::SharedPtr msg);

  /* ROS2发布者 */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_pub_;                                             // 云台关节状态发布者
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lob_shot_pub_;  // 吊射标志发布者

  /* ROS2订阅者 */
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;
  //   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr fire_sub_;

  /* LibXR Topic */
  LibXR::Topic ahrs_quaternion_topic_;
  LibXR::Topic target_euler_topic_;
  LibXR::Topic fire_notify_topic_;
  LibXR::Topic lob_shot_topic_;
  LibXR::Topic target_num_topic_;

  /* LibXR初始化相关成员变量 */
  std::unique_ptr<LibXR::RamFS> ramfs_;
  std::unique_ptr<LibXR::LinuxUART> uart_client_;
  std::unique_ptr<LibXR::Terminal<1024, 64, 16, 128>> terminal_;
  std::unique_ptr<LibXR::Thread> term_thread_;
  std::unique_ptr<LibXR::HardwareContainer> peripherals_;

  /* 云台姿态打印频率相关变量 */
  int ahrs_receive_cnt_ = 0;
  int ahrs_print_freq_ = 50;

  uint8_t last_lob_val_{0};
  bool is_hero_{false};
  bool is_send_vel_ = false;

  std::chrono::time_point<std::chrono::steady_clock,
                          std::chrono::duration<uint64_t, std::ratio<1, 1000000000>>>
      last_send_time_{};
};

}  // namespace rm_serial_driver
#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_