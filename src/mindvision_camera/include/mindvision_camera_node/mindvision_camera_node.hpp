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

#include "CameraApi.h"
#include "CameraDefine.h"
#include "CameraStatus.h"

namespace MindVisionCamera
{
class MindVisionCameraNode : public rclcpp::Node
{
 public:
  explicit MindVisionCameraNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MindVisionCameraNode() override;

 private:
  enum class CameraStateEnum : uint8_t
  {
    STOPPED,
    RUNNING
  };

  struct Parameters
  {
    double exposure_time;  // us
    double gain;           // MindVision raw analog-gain value; clamped to camera range
    int gamma;             // MindVision raw gamma value; clamped to camera range
    bool autocap;          // true: continuous acquisition; false: external trigger
    bool frame_rate_enable;
    double frame_rate;
    double fps_stat_period;    // s
    uint32_t grab_timeout_ms;  // ms, timeout for CameraGetImageBuffer
    uint32_t image_node_num;   // SDK buffer count via CameraSetSysOption("NumBuffers", N)
    std::string frame_id;
    std::string frame_id_lob;
    std::string camera_name;
    std::string camera_name_lob;
    std::string camera_info_url;
    std::string camera_info_url_lob;
    std::string device_sn;
    std::string device_sn_lob;
    uint8_t device_index{0};
    uint8_t device_index_lob{1};
    uint8_t rotate{0};
  };

  struct Protect
  {
    std::mutex mux;
    std::condition_variable is_quit;
    std::thread protect_thread;
  };

  bool InitializeSdkOnce();
  bool Read(cv::Mat& image, rclcpp::Time& stamp);
  void CaptureInit();
  void CaptureStop();
  void ProtectRunning();
  void SwitchCamera(bool to_lob);
  void ReportFpsStats();
  void RunPeriodicManualWhiteBalance();

  void SetExposureTime(double value_us);
  void SetAnalogGain(double value);
  void SetGamma(int value);
  bool CheckStatus(CameraSdkStatus status, const std::string& action, bool fatal = false);
  int SelectDeviceIndex(const tSdkCameraDevInfo* device_list, int device_count) const;
  bool IsTriggerModeSupported(int mode) const;
  static uint8_t ClampUInt8(int value, uint8_t fallback, const rclcpp::Logger& logger,
                            const std::string& name);
  static std::string PhaseError(CameraSdkStatus status);

  Parameters params_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::vector<uint8_t> rotate_buffer_;
  std::vector<uint8_t> mono_buffer_;

  CameraHandle handle_{-1};
  tSdkCameraCapbility capability_{};

  std::atomic<CameraStateEnum> camera_state_{CameraStateEnum::STOPPED};
  std::atomic<bool> running_{true};

  std::thread capture_thread_;
  Protect guard_;

  image_transport::CameraPublisher camera_pub_;

  bool is_lob_camera_{false};
  bool is_hero_{false};

  uint8_t current_device_index_{0};
  std::string current_device_sn_;
  std::string current_frame_id_;
  std::string current_camera_name_;
  std::string current_camera_info_url_;

  std::atomic<bool> in_read_{false};
  std::atomic<bool> is_switching_{false};
  std::atomic<bool> periodic_manual_white_balance_enabled_{false};
  std::atomic<uint32_t> manual_white_balance_frame_count_{0};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lob_shot_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_switch_done_pub_;

  std::atomic<uint64_t> received_frame_count_{0};
  std::atomic<uint64_t> published_frame_count_{0};
  std::chrono::steady_clock::time_point fps_stat_last_time_;
  rclcpp::TimerBase::SharedPtr fps_stat_timer_;
};
}  // namespace MindVisionCamera
