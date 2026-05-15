#include "mindvision_camera_node/mindvision_camera_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <mutex>
#include <unordered_map>

using namespace std::chrono_literals;

namespace
{
constexpr uint32_t kManualWhiteBalanceFramePeriod = 200;
}

namespace MindVisionCamera
{
uint8_t MindVisionCameraNode::ClampUInt8(int value, uint8_t fallback,
                                         const rclcpp::Logger& logger,
                                         const std::string& name)
{
  if (value < 0 || value > 255)
  {
    RCLCPP_WARN(logger, "%s must be in [0, 255]. Use %u instead.", name.c_str(),
                static_cast<unsigned int>(fallback));
    return fallback;
  }
  return static_cast<uint8_t>(value);
}

MindVisionCameraNode::MindVisionCameraNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mindvision_camera_node", options)
{
  params_.exposure_time = this->declare_parameter<double>("exposure_time", 1000.0);
  params_.gain = this->declare_parameter<double>("gain", 15.0);
  params_.gamma = this->declare_parameter<int>("gamma", 75);
  params_.autocap = this->declare_parameter<bool>("autocap", true);
  params_.frame_rate_enable = this->declare_parameter<bool>("frame_rate_enable", false);
  params_.frame_rate = this->declare_parameter<double>("frame_rate", 249.0);
  params_.fps_stat_period = this->declare_parameter<double>("fps_stat_period", 1.0);

  auto grab_timeout_ms_param = this->declare_parameter<int>("grab_timeout_ms", 20);
  auto image_node_num_param = this->declare_parameter<int>("image_node_num", 1);
  params_.grab_timeout_ms = static_cast<uint32_t>(std::max(1L, grab_timeout_ms_param));
  params_.image_node_num = static_cast<uint32_t>(std::max(1L, image_node_num_param));
  if (grab_timeout_ms_param <= 0)
  {
    RCLCPP_WARN(this->get_logger(),
                "grab_timeout_ms must be greater than 0. Use 20 ms instead.");
    params_.grab_timeout_ms = 20;
  }
  if (image_node_num_param <= 0)
  {
    RCLCPP_WARN(this->get_logger(),
                "image_node_num must be greater than 0. Use 1 instead.");
    params_.image_node_num = 1;
  }
  if (params_.fps_stat_period <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(),
                "fps_stat_period must be greater than 0. Use 1.0 s instead.");
    params_.fps_stat_period = 1.0;
  }

  current_frame_id_ = params_.frame_id =
      this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
  current_camera_name_ = params_.camera_name =
      this->declare_parameter<std::string>("camera_name", "gimbal_camera");

  params_.rotate = ClampUInt8(this->declare_parameter<int>("rotate", 0), 0,
                              this->get_logger(), "rotate");
  current_device_index_ = params_.device_index =
      ClampUInt8(this->declare_parameter<int>("device_index", 0), 0, this->get_logger(),
                 "device_index");
  current_device_sn_ = params_.device_sn =
      this->declare_parameter<std::string>("device_sn", "");

  const auto& robot_type = this->declare_parameter<std::string>("robot_type", "infantry");
  is_hero_ = (robot_type == "hero");

  RCLCPP_INFO(this->get_logger(), "params has been initialized.");

  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw",
                                                         rmw_qos_profile_sensor_data);
  RCLCPP_INFO(this->get_logger(), "Camera publisher created.");

  fps_stat_last_time_ = std::chrono::steady_clock::now();
  auto fps_stat_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(std::max(0.001, params_.fps_stat_period)));
  fps_stat_timer_ = this->create_wall_timer(
      fps_stat_period, std::bind(&MindVisionCameraNode::ReportFpsStats, this));
  RCLCPP_INFO(this->get_logger(), "FPS statistics enabled, period: %.3f s.",
              params_.fps_stat_period);

  CaptureInit();
  RCLCPP_INFO(this->get_logger(), "Camera initialized.");

  guard_.protect_thread = std::thread(&MindVisionCameraNode::ProtectRunning, this);

  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, params_.camera_name);
  current_camera_info_url_ = params_.camera_info_url = this->declare_parameter(
      "camera_info_url", "package://mindvision_camera/config/camera_info.yaml");

  if (camera_info_manager_->validateURL(current_camera_info_url_))
  {
    camera_info_manager_->loadCameraInfo(current_camera_info_url_);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                current_camera_info_url_.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Guard thread created.");

  if (is_hero_)
  {
    RCLCPP_WARN(this->get_logger(),
                "Running on robot type: %s, LOB camera support enabled.",
                robot_type.c_str());
    params_.camera_name_lob =
        this->declare_parameter<std::string>("camera_name_lob", "gimbal_camera_lob");
    params_.frame_id_lob =
        this->declare_parameter<std::string>("frame_id_lob", "camera_optical_frame_lob");
    params_.device_index_lob =
        ClampUInt8(this->declare_parameter<int>("device_index_lob", 1), 1,
                   this->get_logger(), "device_index_lob");
    params_.device_sn_lob = this->declare_parameter<std::string>("device_sn_lob", "");
    params_.camera_info_url_lob = this->declare_parameter(
        "camera_info_url_lob", "package://mindvision_camera/config/camera_info_lob.yaml");

    camera_switch_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/camera_switch_done", rclcpp::QoS(1).reliable());

    lob_shot_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/lob_shot_switch", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          if (!msg->data)
          {
            return;
          }
          SwitchCamera(!is_lob_camera_);
        });
  }

  capture_thread_ = std::thread(
      [this]()
      {
        RCLCPP_INFO(this->get_logger(), "MindVision SDK capture thread started.");

        while (running_.load())
        {
          if (camera_state_.load() == CameraStateEnum::STOPPED)
          {
            std::this_thread::sleep_for(10ms);
            continue;
          }

          cv::Mat image;
          rclcpp::Time stamp;
          bool ok = Read(image, stamp);
          if (!ok || image.empty())
          {
            continue;
          }

          uint32_t publish_height = static_cast<uint32_t>(image.rows);
          uint32_t publish_width = static_cast<uint32_t>(image.cols);
          uint32_t publish_step = static_cast<uint32_t>(image.cols * image.elemSize());

          switch (params_.rotate)
          {
            case 1:
            case 3:
            {
              size_t byte_count = image.total() * image.elemSize();
              rotate_buffer_.resize(byte_count);
              cv::Mat rotated(image.cols, image.rows, image.type(), rotate_buffer_.data(),
                              static_cast<size_t>(image.rows) * image.elemSize());
              cv::rotate(image, rotated,
                         params_.rotate == 1 ? cv::ROTATE_90_CLOCKWISE
                                             : cv::ROTATE_90_COUNTERCLOCKWISE);
              image_msg_.data.swap(rotate_buffer_);
              publish_height = static_cast<uint32_t>(rotated.rows);
              publish_width = static_cast<uint32_t>(rotated.cols);
              publish_step = static_cast<uint32_t>(rotated.cols * rotated.elemSize());
              break;
            }
            case 2:
              cv::flip(image, image, -1);
              break;
            default:
              break;
          }

          image_msg_.height = publish_height;
          image_msg_.width = publish_width;
          image_msg_.encoding = "rgb8";
          image_msg_.is_bigendian = false;
          image_msg_.step = publish_step;
          image_msg_.header.stamp = stamp;
          image_msg_.header.frame_id = current_frame_id_;

          camera_info_msg_.height = publish_height;
          camera_info_msg_.width = publish_width;
          camera_info_msg_.header.stamp = stamp;
          camera_info_msg_.header.frame_id = current_frame_id_;

          camera_pub_.publish(image_msg_, camera_info_msg_);
          published_frame_count_.fetch_add(1, std::memory_order_relaxed);
        }

        RCLCPP_INFO(this->get_logger(), "MindVision SDK capture thread exit.");
      });
}

MindVisionCameraNode::~MindVisionCameraNode()
{
  RCLCPP_INFO(this->get_logger(), "Destroying MindVisionCameraNode...");

  running_.store(false);
  guard_.is_quit.notify_all();

  if (capture_thread_.joinable())
  {
    capture_thread_.join();
  }

  CaptureStop();

  if (guard_.protect_thread.joinable())
  {
    guard_.protect_thread.join();
  }

  RCLCPP_INFO(this->get_logger(), "MindVisionCameraNode destroyed.");
}

bool MindVisionCameraNode::InitializeSdkOnce()
{
  static std::once_flag sdk_init_flag;
  static CameraSdkStatus sdk_init_status = CAMERA_STATUS_SUCCESS;
  std::call_once(sdk_init_flag, []() { sdk_init_status = CameraSdkInit(1); });

  if (sdk_init_status != CAMERA_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "CameraSdkInit failed: %d", sdk_init_status);
    return false;
  }
  return true;
}

bool MindVisionCameraNode::Read(cv::Mat& img, rclcpp::Time& timestamp)
{
  in_read_.store(true, std::memory_order_release);

  if (camera_state_.load(std::memory_order_acquire) == CameraStateEnum::STOPPED ||
      handle_ < 0)
  {
    in_read_.store(false, std::memory_order_release);
    return false;
  }

  tSdkFrameHead frame_info{};
  BYTE* raw_buffer = nullptr;
  CameraSdkStatus ret = CameraGetImageBuffer(handle_, &frame_info, &raw_buffer,
                                             static_cast<INT>(params_.grab_timeout_ms));

  if (ret != CAMERA_STATUS_SUCCESS)
  {
    in_read_.store(false, std::memory_order_release);

    if (ret == CAMERA_STATUS_TIME_OUT || ret == CAMERA_STATUS_WAIT)
    {
      return false;
    }

    RCLCPP_ERROR(this->get_logger(),
                 "CameraGetImageBuffer failed: %d, switching to Stopped.", ret);
    camera_state_.store(CameraStateEnum::STOPPED, std::memory_order_release);
    guard_.is_quit.notify_all();
    return false;
  }

  timestamp = this->now();

  int width = frame_info.iWidth;
  int height = frame_info.iHeight;
  if (width <= 0 || height <= 0)
  {
    CameraReleaseImageBuffer(handle_, raw_buffer);
    in_read_.store(false, std::memory_order_release);
    return false;
  }
  size_t rgb_byte_count = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
  image_msg_.data.resize(rgb_byte_count);
  cv::Mat dst_image(height, width, CV_8UC3, image_msg_.data.data(),
                    static_cast<size_t>(width) * 3);

  bool process_ok = true;
  CameraSdkStatus process_ret = CAMERA_STATUS_SUCCESS;

  process_ret =
      CameraImageProcess(handle_, raw_buffer, image_msg_.data.data(), &frame_info);

  if (process_ret != CAMERA_STATUS_SUCCESS)
  {
    process_ok = false;
  }

  CameraSdkStatus release_ret = CameraReleaseImageBuffer(handle_, raw_buffer);

  if (!process_ok)
  {
    in_read_.store(false, std::memory_order_release);
    RCLCPP_ERROR(this->get_logger(), "CameraImageProcess failed: %d", process_ret);
    camera_state_.store(CameraStateEnum::STOPPED, std::memory_order_release);
    guard_.is_quit.notify_all();
    return false;
  }

  if (release_ret != CAMERA_STATUS_SUCCESS)
  {
    in_read_.store(false, std::memory_order_release);
    RCLCPP_ERROR(this->get_logger(),
                 "CameraReleaseImageBuffer failed: %d, switching to Stopped.",
                 release_ret);
    camera_state_.store(CameraStateEnum::STOPPED, std::memory_order_release);
    guard_.is_quit.notify_all();
    return false;
  }

  img = dst_image;
  RunPeriodicManualWhiteBalance();
  in_read_.store(false, std::memory_order_release);
  received_frame_count_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void MindVisionCameraNode::RunPeriodicManualWhiteBalance()
{
  if (!periodic_manual_white_balance_enabled_.load(std::memory_order_relaxed) ||
      handle_ < 0)
  {
    return;
  }

  uint32_t frame_count =
      manual_white_balance_frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;
  if (frame_count < kManualWhiteBalanceFramePeriod)
  {
    return;
  }

  manual_white_balance_frame_count_.store(0, std::memory_order_relaxed);
  CheckStatus(CameraSetOnceWB(handle_), "CameraSetOnceWB(periodic)", false);
}

void MindVisionCameraNode::ReportFpsStats()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration<double>(now - fps_stat_last_time_).count();
  fps_stat_last_time_ = now;

  if (elapsed <= 0.0)
  {
    return;
  }

  auto received = received_frame_count_.exchange(0, std::memory_order_relaxed);
  auto published = published_frame_count_.exchange(0, std::memory_order_relaxed);
  double received_fps = static_cast<double>(received) / elapsed;
  double published_fps = static_cast<double>(published) / elapsed;

  RCLCPP_INFO(this->get_logger(),
              "Camera FPS stats: received %.2f Hz (%lu frames), published %.2f Hz (%lu "
              "frames), period %.3f s",
              received_fps, static_cast<unsigned long>(received), published_fps,
              static_cast<unsigned long>(published), elapsed);
}

bool MindVisionCameraNode::CheckStatus(CameraSdkStatus status, const std::string& action,
                                       bool fatal)
{
  if (status == CAMERA_STATUS_SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "%s succeeded.", action.c_str());
    return true;
  }

  if (fatal)
  {
    RCLCPP_ERROR(this->get_logger(), "%s failed: %s", action.c_str(),
                 PhaseError(status).c_str());
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "%s failed: %s", action.c_str(),
                PhaseError(status).c_str());
  }
  return false;
}

int MindVisionCameraNode::SelectDeviceIndex(const tSdkCameraDevInfo* device_list,
                                            int device_count) const
{
  if (!current_device_sn_.empty())
  {
    for (int i = 0; i < device_count; ++i)
    {
      if (current_device_sn_ == device_list[i].acSn)
      {
        return i;
      }
    }
    RCLCPP_ERROR(this->get_logger(), "Device SN '%s' not found among %d cameras.",
                 current_device_sn_.c_str(), device_count);
    return -1;
  }

  if (current_device_index_ >= static_cast<uint8_t>(device_count))
  {
    RCLCPP_ERROR(this->get_logger(), "Device index %d out of range (found %d cameras)",
                 current_device_index_, device_count);
    return -1;
  }
  return static_cast<int>(current_device_index_);
}

bool MindVisionCameraNode::IsTriggerModeSupported(int mode) const
{
  if (capability_.iTriggerDesc <= 0 || capability_.pTriggerDesc == nullptr)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Camera capability has no trigger-mode list; skip trigger-mode precheck.");
    return true;
  }

  for (int i = 0; i < capability_.iTriggerDesc; ++i)
  {
    if (capability_.pTriggerDesc[i].iIndex == mode)
    {
      return true;
    }
  }
  return false;
}

void MindVisionCameraNode::CaptureInit()
{
  if (!running_.load())
  {
    return;
  }
  if (!InitializeSdkOnce())
  {
    return;
  }

  std::string num_buffers = std::to_string(params_.image_node_num);
  CameraSdkStatus ret = CameraSetSysOption("NumBuffers", num_buffers.c_str());
  CheckStatus(ret, "CameraSetSysOption(NumBuffers)", false);

  constexpr int kMaxCameraCount = 16;
  tSdkCameraDevInfo device_list[kMaxCameraCount]{};
  INT device_count = kMaxCameraCount;
  ret = CameraEnumerateDevice(device_list, &device_count);
  if (!CheckStatus(ret, "CameraEnumerateDevice", true))
  {
    return;
  }

  if (device_count == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Not found camera!");
    return;
  }

  for (int i = 0; i < device_count; ++i)
  {
    RCLCPP_INFO(this->get_logger(),
                "MindVision camera[%d]: product=%s, friendly=%s, sn=%s, port=%s", i,
                device_list[i].acProductName, device_list[i].acFriendlyName,
                device_list[i].acSn, device_list[i].acPortType);
  }

  int selected_index = SelectDeviceIndex(device_list, device_count);
  if (selected_index < 0)
  {
    return;
  }

  ret = CameraInit(&device_list[selected_index], -1, -1, &handle_);
  if (!CheckStatus(ret, "CameraInit", true))
  {
    handle_ = -1;
    return;
  }

  ret = CameraGetCapability(handle_, &capability_);
  if (!CheckStatus(ret, "CameraGetCapability", true))
  {
    CaptureStop();
    return;
  }

  periodic_manual_white_balance_enabled_.store(false, std::memory_order_relaxed);
  manual_white_balance_frame_count_.store(0, std::memory_order_relaxed);

  int max_width = capability_.sResolutionRange.iWidthMax;
  int max_height = capability_.sResolutionRange.iHeightMax;
  if (max_width > 0 && max_height > 0)
  {
    image_msg_.data.reserve(static_cast<size_t>(max_width) *
                            static_cast<size_t>(max_height) * 3);
    image_msg_.height = static_cast<uint32_t>(max_height);
    image_msg_.width = static_cast<uint32_t>(max_width);
  }

  int trigger_mode = params_.autocap ? CONTINUATION : EXTERNAL_TRIGGER;
  if (!IsTriggerModeSupported(trigger_mode))
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Requested trigger mode %d is not supported by this camera.",
                 trigger_mode);
    CaptureStop();
    return;
  }

  ret = CameraSetTriggerMode(handle_, trigger_mode);
  if (!CheckStatus(ret,
                   params_.autocap ? "CameraSetTriggerMode(CONTINUATION)"
                                   : "CameraSetTriggerMode(EXTERNAL_TRIGGER)",
                   true))
  {
    CaptureStop();
    return;
  }

  if (!params_.autocap)
  {
    CheckStatus(CameraSetExtTrigSignalType(handle_, EXT_TRIG_LEADING_EDGE),
                "CameraSetExtTrigSignalType(EXT_TRIG_LEADING_EDGE)", false);
    CheckStatus(CameraSetTriggerCount(handle_, 1), "CameraSetTriggerCount(1)", false);
  }
  CameraSdkStatus lut_ret = CameraSetLutMode(handle_, LUTMODE_PARAM_GEN);
  if (CheckStatus(lut_ret, "CameraSetLutMode(LUTMODE_PARAM_GEN)", false))
  {
    SetGamma(params_.gamma);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
                "Gamma will not take effect because LUT dynamic parameter mode "
                "could not be enabled.");
  }

  CameraSdkStatus wb_ret = CameraSetWbMode(handle_, TRUE);
  if (!CheckStatus(wb_ret, "CameraSetWbMode(auto)", false))
  {
    if (capability_.sIspCapacity.bWbOnce)
    {
      periodic_manual_white_balance_enabled_.store(true, std::memory_order_relaxed);
      RCLCPP_WARN(this->get_logger(),
                  "Auto white balance failed; will run one-shot manual white "
                  "balance every %u frames.",
                  static_cast<unsigned int>(kManualWhiteBalanceFramePeriod));
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "Auto white balance failed and one-shot manual white balance is "
                  "not supported by this camera.");
    }
  }

  ret = CameraSetIspOutFormat(handle_, CAMERA_MEDIA_TYPE_RGB8);

  if (!CheckStatus(ret, "CameraSetIspOutFormat", true))
  {
    CaptureStop();
    return;
  }

  CheckStatus(CameraSetAeState(handle_, FALSE), "CameraSetAeState(false)", false);
  SetExposureTime(params_.exposure_time);
  SetAnalogGain(params_.gain);

  if (capability_.iFrameSpeedDesc > 0)
  {
    int max_speed_index = capability_.iFrameSpeedDesc - 1;
    CheckStatus(CameraSetFrameSpeed(handle_, max_speed_index), "CameraSetFrameSpeed(max)",
                false);
  }

  if (params_.frame_rate_enable)
  {
    int fps = static_cast<int>(std::round(params_.frame_rate));
    if (fps <= 0)
    {
      RCLCPP_INFO(this->get_logger(),
                  "frame_rate <= 0 requested; SDK interprets this as maximum rate.");
    }
    CheckStatus(CameraSetFrameRate(handle_, fps), "CameraSetFrameRate", false);
  }

  ret = CameraPlay(handle_);
  if (!CheckStatus(ret, "CameraPlay", true))
  {
    CaptureStop();
    return;
  }

  CheckStatus(CameraClearBuffer(handle_), "CameraClearBuffer", false);

  camera_state_.store(CameraStateEnum::RUNNING);
  RCLCPP_INFO(this->get_logger(), "MindVision camera initialized and started.");
}

void MindVisionCameraNode::CaptureStop()
{
  camera_state_.store(CameraStateEnum::STOPPED);
  periodic_manual_white_balance_enabled_.store(false, std::memory_order_relaxed);
  manual_white_balance_frame_count_.store(0, std::memory_order_relaxed);

  if (handle_ < 0)
  {
    return;
  }

  CheckStatus(CameraStop(handle_), "CameraStop", false);
  CheckStatus(CameraUnInit(handle_), "CameraUnInit", false);

  handle_ = -1;
  RCLCPP_INFO(this->get_logger(), "MindVision camera stopped and handle destroyed.");
}

void MindVisionCameraNode::ProtectRunning()
{
  RCLCPP_INFO(this->get_logger(), "Protect thread started.");

  std::unique_lock<std::mutex> lock(this->guard_.mux);
  while (running_.load())
  {
    this->guard_.is_quit.wait(
        lock,
        [this]
        {
          return (this->camera_state_.load() == CameraStateEnum::STOPPED &&
                  !this->is_switching_.load()) ||
                 (!this->running_.load());
        });

    if (!this->running_.load())
    {
      break;
    }

    RCLCPP_INFO(this->get_logger(), "Camera stopped, attempting to restart...");
    this->CaptureStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    this->CaptureInit();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(this->get_logger(), "Protect thread exit.");
}

void MindVisionCameraNode::SwitchCamera(bool to_lob)
{
  is_switching_.store(true, std::memory_order_seq_cst);
  RCLCPP_INFO(this->get_logger(), "Switching to %s camera...", to_lob ? "lob" : "normal");

  camera_state_.store(CameraStateEnum::STOPPED, std::memory_order_seq_cst);

  while (in_read_.load(std::memory_order_seq_cst))
  {
    std::this_thread::yield();
  }

  CaptureStop();

  current_device_index_ = to_lob ? params_.device_index_lob : params_.device_index;
  current_device_sn_ = to_lob ? params_.device_sn_lob : params_.device_sn;
  current_camera_info_url_ =
      to_lob ? params_.camera_info_url_lob : params_.camera_info_url;
  current_camera_name_ = to_lob ? params_.camera_name_lob : params_.camera_name;
  current_frame_id_ = to_lob ? params_.frame_id_lob : params_.frame_id;

  camera_info_manager_->setCameraName(current_camera_name_);
  if (camera_info_manager_->validateURL(current_camera_info_url_))
  {
    camera_info_manager_->loadCameraInfo(current_camera_info_url_);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
    RCLCPP_INFO(this->get_logger(), "Loaded camera info: %s",
                current_camera_info_url_.c_str());
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL for %s: %s",
                to_lob ? "lob" : "normal", current_camera_info_url_.c_str());
  }

  CaptureInit();

  if (camera_state_.load() == CameraStateEnum::RUNNING)
  {
    is_lob_camera_ = to_lob;
    std_msgs::msg::Bool done_msg;
    done_msg.data = to_lob;
    camera_switch_done_pub_->publish(done_msg);
    RCLCPP_INFO(this->get_logger(), "Camera switched to %s successfully.",
                to_lob ? "lob" : "normal");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),
                 "CaptureInit failed after switching to %s camera, state not updated.",
                 to_lob ? "lob" : "normal");
  }

  is_switching_.store(false, std::memory_order_seq_cst);
}

void MindVisionCameraNode::SetExposureTime(double value_us)
{
  if (handle_ < 0)
  {
    return;
  }

  double min_us = 0.0;
  double max_us = 0.0;
  double step_us = 0.0;
  CameraSdkStatus range_ret =
      CameraGetExposureTimeRange(handle_, &min_us, &max_us, &step_us);
  if (range_ret == CAMERA_STATUS_SUCCESS && max_us >= min_us)
  {
    double clamped_value = std::max(min_us, std::min(max_us, value_us));
    if (std::fabs(clamped_value - value_us) > 1e-6)
    {
      RCLCPP_WARN(this->get_logger(),
                  "exposure_time %.3f us is out of camera range [%.3f, %.3f] us. "
                  "Use %.3f us instead.",
                  value_us, min_us, max_us, clamped_value);
    }
    value_us = clamped_value;
  }
  else
  {
    CheckStatus(range_ret, "CameraGetExposureTimeRange", false);
  }

  CameraSdkStatus ret = CameraSetExposureTime(handle_, value_us);
  if (ret != CAMERA_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "CameraSetExposureTime(%f us) failed: %d", value_us,
                 ret);
    return;
  }

  double actual_us = 0.0;
  CameraSdkStatus get_ret = CameraGetExposureTime(handle_, &actual_us);
  if (get_ret == CAMERA_STATUS_SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Exposure time requested %.3f us, actual %.3f us.",
                value_us, actual_us);
  }
  else
  {
    CheckStatus(get_ret, "CameraGetExposureTime", false);
  }
}

void MindVisionCameraNode::SetAnalogGain(double value)
{
  if (handle_ < 0)
  {
    return;
  }

  int min_gain = static_cast<int>(capability_.sExposeDesc.uiAnalogGainMin);
  int max_gain = static_cast<int>(capability_.sExposeDesc.uiAnalogGainMax);
  int requested_gain = static_cast<int>(std::round(value));
  int gain = requested_gain;
  if (max_gain >= min_gain)
  {
    gain = std::max(min_gain, std::min(max_gain, gain));
    if (gain != requested_gain)
    {
      RCLCPP_WARN(this->get_logger(),
                  "gain %.3f is out of camera range [%d, %d]. Use %d instead.", value,
                  min_gain, max_gain, gain);
    }
  }

  CameraSdkStatus ret = CameraSetAnalogGain(handle_, gain);
  if (ret != CAMERA_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "CameraSetAnalogGain(%d, requested %.3f) failed: %d",
                 gain, value, ret);
  }
}

void MindVisionCameraNode::SetGamma(int value)
{
  if (handle_ < 0)
  {
    return;
  }

  int min_gamma = capability_.sGammaRange.iMin;
  int max_gamma = capability_.sGammaRange.iMax;
  int gamma = value;
  if (max_gamma >= min_gamma)
  {
    gamma = std::max(min_gamma, std::min(max_gamma, gamma));
    if (gamma != value)
    {
      RCLCPP_WARN(this->get_logger(),
                  "gamma %d is out of camera range [%d, %d]. Use %d instead.", value,
                  min_gamma, max_gamma, gamma);
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
                "Camera reported invalid gamma range [%d, %d]; use requested %d.",
                min_gamma, max_gamma, gamma);
  }

  CameraSdkStatus ret = CameraSetGamma(handle_, gamma);
  if (ret != CAMERA_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "CameraSetGamma(%d, requested %d) failed: %s", gamma,
                 value, PhaseError(ret).c_str());
    return;
  }

  int actual_gamma = 0;
  CameraSdkStatus get_ret = CameraGetGamma(handle_, &actual_gamma);
  if (get_ret == CAMERA_STATUS_SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(),
                "Gamma requested %d, applied %d, actual %d, camera range [%d, %d].",
                value, gamma, actual_gamma, min_gamma, max_gamma);
  }
  else
  {
    CheckStatus(get_ret, "CameraGetGamma", false);
  }
}

std::string MindVisionCameraNode::PhaseError(CameraSdkStatus status)
{
  static const std::unordered_map<int, std::string> ERROR_MAP = {
      {CAMERA_STATUS_SUCCESS, "操作成功"},
      {CAMERA_STATUS_FAILED, "操作失败"},
      {CAMERA_STATUS_INTERNAL_ERROR, "内部错误"},
      {CAMERA_STATUS_UNKNOW, "未知错误"},
      {CAMERA_STATUS_NOT_SUPPORTED, "不支持该功能"},
      {CAMERA_STATUS_NOT_INITIALIZED, "初始化未完成"},
      {CAMERA_STATUS_PARAMETER_INVALID, "参数无效"},
      {CAMERA_STATUS_PARAMETER_OUT_OF_BOUND, "参数越界"},
      {CAMERA_STATUS_UNENABLED, "未使能"},
      {CAMERA_STATUS_USER_CANCEL, "用户手动取消"},
      {CAMERA_STATUS_PATH_NOT_FOUND, "注册表中没有找到对应的路径"},
      {CAMERA_STATUS_SIZE_DISMATCH, "获得图像数据长度和定义的尺寸不匹配"},
      {CAMERA_STATUS_TIME_OUT, "超时错误"},
      {CAMERA_STATUS_IO_ERROR, "硬件IO错误"},
      {CAMERA_STATUS_COMM_ERROR, "通讯错误"},
      {CAMERA_STATUS_BUS_ERROR, "总线错误"},
      {CAMERA_STATUS_NO_DEVICE_FOUND, "没有发现设备"},
      {CAMERA_STATUS_NO_LOGIC_DEVICE_FOUND, "未找到逻辑设备"},
      {CAMERA_STATUS_DEVICE_IS_OPENED, "设备已经打开"},
      {CAMERA_STATUS_DEVICE_IS_CLOSED, "设备已经关闭"},
      {CAMERA_STATUS_DEVICE_VEDIO_CLOSED, "没有打开设备视频"},
      {CAMERA_STATUS_NO_MEMORY, "没有足够系统内存"},
      {CAMERA_STATUS_FILE_CREATE_FAILED, "创建文件失败"},
      {CAMERA_STATUS_FILE_INVALID, "文件格式无效"},
      {CAMERA_STATUS_WRITE_PROTECTED, "写保护，不可写"},
      {CAMERA_STATUS_GRAB_FAILED, "数据采集失败"},
      {CAMERA_STATUS_LOST_DATA, "数据丢失，不完整"},
      {CAMERA_STATUS_EOF_ERROR, "未接收到帧结束符"},
      {CAMERA_STATUS_BUSY, "正忙(上一次操作还在进行中)"},
      {CAMERA_STATUS_WAIT, "需要等待，可以再次尝试"},
      {CAMERA_STATUS_IN_PROCESS, "正在进行，已经被操作过"},
      {CAMERA_STATUS_IIC_ERROR, "IIC传输错误"},
      {CAMERA_STATUS_SPI_ERROR, "SPI传输错误"},
      {CAMERA_STATUS_USB_CONTROL_ERROR, "USB控制传输错误"},
      {CAMERA_STATUS_USB_BULK_ERROR, "USB BULK传输错误"},
      {CAMERA_STATUS_SOCKET_INIT_ERROR, "网络传输套件初始化失败"},
      {CAMERA_STATUS_GIGE_FILTER_INIT_ERROR, "网络相机内核过滤驱动初始化失败"},
      {CAMERA_STATUS_NET_SEND_ERROR, "网络数据发送错误"},
      {CAMERA_STATUS_DEVICE_LOST, "与网络相机失去连接，心跳检测超时"},
      {CAMERA_STATUS_DATA_RECV_LESS, "接收到的字节数比请求的少"},
      {CAMERA_STATUS_FUNCTION_LOAD_FAILED, "从文件中加载程序失败"},
      {CAMERA_STATUS_CRITICAL_FILE_LOST, "程序运行所必须的文件丢失"},
      {CAMERA_STATUS_SENSOR_ID_DISMATCH, "固件和程序不匹配"},
      {CAMERA_STATUS_OUT_OF_RANGE, "参数超出有效范围"},
      {CAMERA_STATUS_REGISTRY_ERROR, "安装程序注册错误"},
      {CAMERA_STATUS_ACCESS_DENY, "禁止访问"},
      {CAMERA_STATUS_CAMERA_NEED_RESET, "相机需要复位后才能正常使用"},
      {CAMERA_STATUS_ISP_MOUDLE_NOT_INITIALIZED, "ISP模块未初始化"},
      {CAMERA_STATUS_ISP_DATA_CRC_ERROR, "数据校验错误"},
      {CAMERA_STATUS_MV_TEST_FAILED, "数据测试失败"},
      {CAMERA_STATUS_INTERNAL_ERR1, "内部错误1"},
      {CAMERA_STATUS_U3V_NO_CONTROL_EP, "U3V控制端点未找到"},
      {CAMERA_STATUS_U3V_CONTROL_ERROR, "U3V控制通讯错误"},
      {CAMERA_STATUS_INVALID_FRIENDLY_NAME, "无效的设备名"},
      {CAMERA_STATUS_FORMAT_ERROR, "格式错误"},
      {CAMERA_STATUS_PCIE_OPEN_ERROR, "PCIE设备打开失败"},
      {CAMERA_STATUS_PCIE_COMM_ERROR, "PCIE设备通讯失败"},
      {CAMERA_STATUS_PCIE_DDR_ERROR, "PCIE DDR错误"},
      {CAMERA_STATUS_IP_ERROR, "IP错误"},
      {CAMERA_AIA_PACKET_RESEND, "该帧需要重传"},
      {CAMERA_AIA_NOT_IMPLEMENTED, "设备不支持的命令"},
      {CAMERA_AIA_INVALID_PARAMETER, "命令参数非法"},
      {CAMERA_AIA_INVALID_ADDRESS, "不可访问的地址"},
      {CAMERA_AIA_WRITE_PROTECT, "访问的对象不可写"},
      {CAMERA_AIA_BAD_ALIGNMENT, "访问的地址没有按照要求对齐"},
      {CAMERA_AIA_ACCESS_DENIED, "没有访问权限"},
      {CAMERA_AIA_BUSY, "命令正在处理中"},
      {CAMERA_AIA_DEPRECATED, "该指令已经废弃"},
      {CAMERA_AIA_PACKET_UNAVAILABLE, "包无效"},
      {CAMERA_AIA_DATA_OVERRUN, "数据溢出"},
      {CAMERA_AIA_INVALID_HEADER, "数据包头部与协议不匹配"},
      {CAMERA_AIA_PACKET_NOT_YET_AVAILABLE, "图像分包数据还未准备好"},
      {CAMERA_AIA_PACKET_AND_PREV_REMOVED_FROM_MEMORY, "需要访问的分包已经不存在"},
      {CAMERA_AIA_PACKET_REMOVED_FROM_MEMORY, "分包已从内存中删除"},
      {CAMERA_AIA_NO_REF_TIME, "没有参考时钟源"},
      {CAMERA_AIA_PACKET_TEMPORARILY_UNAVAILABLE, "由于带宽问题，分包暂时不可用"},
      {CAMERA_AIA_OVERFLOW, "设备端数据溢出"},
      {CAMERA_AIA_ACTION_LATE, "命令执行已经超过有效的指定时间"},
      {CAMERA_AIA_ERROR, "AIA错误"},
      {CAMERA_MV_SSR_COMM_ERROR, "SSR通讯错误"},
      {CAMERA_MV_SSR_TRAIN_ERROR, "SSR训练错误"},
  };

  auto it = ERROR_MAP.find(status);
  if (it != ERROR_MAP.end())
  {
    return it->second;
  }

  return "Unknown error code: 0x" + std::string(status < 0 ? "-" : "") +
         std::to_string(std::abs(status));
}

}  // namespace MindVisionCamera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(MindVisionCamera::MindVisionCameraNode)
