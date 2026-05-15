#pragma once

#include <atomic>
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>
#include <vector>

#include "CameraParams.h"
#include "MvCameraControl.h"
#include "MvErrorDefine.h"
#include "MvISPErrorDefine.h"

namespace HikCamera
{
class HikCameraNode : public rclcpp::Node
{
 public:
  explicit HikCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~HikCameraNode() override;

 private:
  enum class HikStateEnum : uint8_t
  {
    STOPPED,
    RUNNING
  };

  struct Parameters
  {
    double exposure_time;  // us
    double gain;
    bool autocap;
    bool frame_rate_enable;
    double frame_rate;
    double fps_stat_period;    // s
    uint32_t grab_timeout_ms;  // ms, timeout for MV_CC_GetImageBuffer
    uint32_t image_node_num;   // SDK output buffer nodes, lower means lower latency
    std::string frame_id;
    std::string frame_id_lob;
    std::string camera_name;
    std::string camera_name_lob;
    std::string camera_info_url;
    std::string camera_info_url_lob;
    uint8_t device_index{0};
    uint8_t device_index_lob{1};
    uint8_t rotate = 0;
  };

  struct Protect
  {
    std::mutex mux;
    std::condition_variable is_quit;
    std::thread protect_thread;
  };

  // === 核心逻辑 ===
  bool Read(cv::Mat& image, rclcpp::Time& stamp);
  void CaptureInit();
  void CaptureStop();
  void ProtectRunning();
  void SwitchCamera(bool to_lob);
  void ReportFpsStats();

  void SetFloatValue(const std::string& name, double value);
  void SetEnumValue(const std::string& name, unsigned int value);

  static std::string PhaseError(unsigned int error_code);

  // === 参数 ===
  Parameters params_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  MV_IMAGE_BASIC_INFO img_info_;
  std::vector<uint8_t> rotate_buffer_;

  // === SDK 句柄 ===
  void* handle_{nullptr};

  std::atomic<HikStateEnum> hik_state_{HikStateEnum::STOPPED};
  std::atomic<bool> running_{true};

  std::thread capture_thread_;
  Protect guard_;

  // ROS2 publisher
  image_transport::CameraPublisher camera_pub_;

  // Lob shot camera switch
  bool is_lob_camera_{false};
  bool is_hero_{false};

  // Current camera_info state
  uint8_t current_device_index_;
  std::string current_frame_id_;
  std::string current_camera_name_;
  std::string current_camera_info_url_;

  std::atomic<bool> in_read_{false};       // Read() 进入 SDK 前置 true，返回后置 false
  std::atomic<bool> is_switching_{false};  // SwitchCamera 期间为 true，防止守护线程误重启
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lob_shot_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_switch_done_pub_;

  // FPS statistics
  std::atomic<uint64_t> received_frame_count_{0};
  std::atomic<uint64_t> published_frame_count_{0};
  std::chrono::steady_clock::time_point fps_stat_last_time_;
  rclcpp::TimerBase::SharedPtr fps_stat_timer_;
};
}  // namespace HikCamera
