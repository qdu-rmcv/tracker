#include "hik_camera_node/hik_camera_node.hpp"

#include <algorithm>
#include <cstring>
#include <functional>

using namespace std::chrono_literals;

namespace HikCamera
{
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hik_camera_node", options)
{
  params_.exposure_time = this->declare_parameter<double>("exposure_time", 1000.0);  // us
  params_.gain = this->declare_parameter<double>("gain", 15.0);
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
  params_.rotate = this->declare_parameter<uint8_t>("rotate", 0);
  current_device_index_ = params_.device_index =
      this->declare_parameter<uint8_t>("device_index", 0);
  const auto& robot_type = this->declare_parameter<std::string>("robot_type", "infantry");
  is_hero_ = (robot_type == "hero");

  RCLCPP_INFO(this->get_logger(), "params has been initialized.");

  // 创建 publisher
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw",
                                                         rmw_qos_profile_sensor_data);
  RCLCPP_INFO(this->get_logger(), "Camera publisher created.");

  // 创建 FPS 统计定时器，分别统计 SDK 成功取到的帧和成功 publish 的帧
  fps_stat_last_time_ = std::chrono::steady_clock::now();
  auto fps_stat_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(std::max(0.001, params_.fps_stat_period)));
  fps_stat_timer_ = this->create_wall_timer(
      fps_stat_period, std::bind(&HikCameraNode::ReportFpsStats, this));
  RCLCPP_INFO(this->get_logger(), "FPS statistics enabled, period: %.3f s.",
              params_.fps_stat_period);

  // 初始化相机
  CaptureInit();
  RCLCPP_INFO(this->get_logger(), "Camera initialized.");

  // 创建守护线程，负责自动重启
  guard_.protect_thread = std::thread(&HikCameraNode::ProtectRunning, this);

  if (handle_ != nullptr && MV_CC_GetImageInfo(handle_, &img_info_) == MV_OK)
  {
    image_msg_.data.reserve(
        static_cast<size_t>(img_info_.nHeightMax * img_info_.nWidthMax) * 3);
    image_msg_.height = img_info_.nHeightMax;
    image_msg_.width = img_info_.nWidthMax;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
                "Get camera image info failed; message buffer will grow on first frame.");
  }
  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, params_.camera_name);
  current_camera_info_url_ = params_.camera_info_url = this->declare_parameter(
      "camera_info_url", "package://hik_camera/config/camera_info.yaml");

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
    params_.device_index_lob = this->declare_parameter<uint8_t>("device_index_lob", 1);
    params_.camera_info_url_lob = this->declare_parameter(
        "camera_info_url_lob", "package://hik_camera/config/camera_info_lob.yaml");

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

  // 创建取流线程
  capture_thread_ = std::thread(
      [this]()
      {
        RCLCPP_INFO(this->get_logger(), "Hik SDK capture thread started.");

        while (running_.load())
        {
          if (hik_state_.load() == HikStateEnum::STOPPED)
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
              // 180 度可原地翻转，不再额外分配/复制。
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

        RCLCPP_INFO(this->get_logger(), "Hik SDK capture thread exit.");
      });
}

HikCameraNode::~HikCameraNode()
{
  RCLCPP_INFO(this->get_logger(), "Destroying HikCameraNode...");

  running_.store(false);

  // 通知守护线程退出
  guard_.is_quit.notify_all();

  // 先停采集线程
  if (capture_thread_.joinable())
  {
    capture_thread_.join();
  }

  // 关闭相机
  CaptureStop();

  // 再停守护线程
  if (guard_.protect_thread.joinable())
  {
    guard_.protect_thread.join();
  }

  RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed.");
}

bool HikCameraNode::Read(cv::Mat& img, rclcpp::Time& timestamp)
{
  in_read_.store(true, std::memory_order_release);

  if (hik_state_.load(std::memory_order_acquire) == HikStateEnum::STOPPED ||
      handle_ == nullptr)
  {
    in_read_.store(false, std::memory_order_release);
    return false;
  }

  MV_FRAME_OUT raw{};
  unsigned int ret = MV_CC_GetImageBuffer(handle_, &raw, params_.grab_timeout_ms);

  if (ret != MV_OK)
  {
    in_read_.store(false, std::memory_order_release);

    // 短超时只表示当前没有新帧，不能作为掉线处理；否则会频繁重启相机。
    if (ret == MV_E_NODATA || ret == MV_E_NOOUTBUF)
    {
      return false;
    }

    RCLCPP_ERROR(this->get_logger(),
                 "MV_CC_GetImageBuffer failed: 0x%X, switching to Stopped.", ret);
    hik_state_.store(HikStateEnum::STOPPED, std::memory_order_release);
    guard_.is_quit.notify_all();
    return false;
  }

  timestamp = this->now();

  const auto& frame_info = raw.stFrameInfo;
  int width = static_cast<int>(frame_info.nWidth);
  int height = static_cast<int>(frame_info.nHeight);
  if (width <= 0 || height <= 0)
  {
    MV_CC_FreeImageBuffer(handle_, &raw);
    in_read_.store(false, std::memory_order_release);
    return false;
  }

  size_t byte_count = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
  image_msg_.data.resize(byte_count);
  cv::Mat dst_image(height, width, CV_8UC3, image_msg_.data.data(),
                    static_cast<size_t>(width) * 3);

  bool convert_ok = true;
  switch (frame_info.enPixelType)
  {
    case PixelType_Gvsp_BayerGR8:
    {
      cv::Mat raw_img(height, width, CV_8UC1, raw.pBufAddr);
      cv::cvtColor(raw_img, dst_image, cv::COLOR_BayerGR2BGR);
      break;
    }
    case PixelType_Gvsp_BayerRG8:
    {
      cv::Mat raw_img(height, width, CV_8UC1, raw.pBufAddr);
      cv::cvtColor(raw_img, dst_image, cv::COLOR_BayerRG2BGR);
      break;
    }
    case PixelType_Gvsp_BayerGB8:
    {
      cv::Mat raw_img(height, width, CV_8UC1, raw.pBufAddr);
      cv::cvtColor(raw_img, dst_image, cv::COLOR_BayerGB2BGR);
      break;
    }
    case PixelType_Gvsp_BayerBG8:
    {
      cv::Mat raw_img(height, width, CV_8UC1, raw.pBufAddr);
      cv::cvtColor(raw_img, dst_image, cv::COLOR_BayerBG2BGR);
      break;
    }
    default:
      convert_ok = false;
      break;
  }

  unsigned int free_ret = MV_CC_FreeImageBuffer(handle_, &raw);
  in_read_.store(false, std::memory_order_release);

  if (!convert_ok)
  {
    RCLCPP_ERROR(this->get_logger(), "Unsupported pixel type: 0x%X",
                 static_cast<unsigned int>(frame_info.enPixelType));
    hik_state_.store(HikStateEnum::STOPPED, std::memory_order_release);
    guard_.is_quit.notify_all();
    return false;
  }

  if (free_ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "MV_CC_FreeImageBuffer failed: 0x%X, switching to Stopped.", free_ret);
    hik_state_.store(HikStateEnum::STOPPED, std::memory_order_release);
    guard_.is_quit.notify_all();
    return false;
  }

  img = dst_image;
  received_frame_count_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void HikCameraNode::ReportFpsStats()
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

void HikCameraNode::CaptureInit()
{
  if (!running_.load())
  {
    return;
  }
  unsigned int ret{};
  MV_CC_DEVICE_INFO_LIST device_list{};
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_EnumDevices failed: 0x%X", ret);
    return;
  }

  if (device_list.nDeviceNum == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Not found camera!");
    return;
  }

  if (current_device_index_ >= device_list.nDeviceNum)
  {
    RCLCPP_ERROR(this->get_logger(), "Device index %d out of range (found %d cameras)",
                 current_device_index_, device_list.nDeviceNum);
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[current_device_index_]);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_CreateHandle failed: 0x%X", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_OpenDevice failed: 0x%X", ret);
    return;
  }

  unsigned int n_image_node_num = params_.image_node_num;
  ret = MV_CC_SetImageNodeNum(handle_, n_image_node_num);
  if (MV_OK != ret)
  {
    // 设置失败
    RCLCPP_ERROR(this->get_logger(), "MV_CC_SetImageNodeNum failed: 0x%X", ret);
    return;
  }

  if (!params_.autocap)
  {
    ret = MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous");
    if (MV_OK != ret)
    {
      RCLCPP_ERROR(this->get_logger(), "Set Acquisition Mode to Continuous fail! 0x%X",
                   ret);
      return;
    }

    //    将触发模式设置为开启 (On)
    //    参数 "TriggerMode" 的值: 0 表示 Off, 1 表示 On
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
    if (MV_OK != ret)
    {
      RCLCPP_ERROR(this->get_logger(), "Set Trigger Mode to On fail! 0x%X", ret);
      return;
    }

    //    设置触发源为外部硬件触发 (Line0)
    //    可用的值通常有 "Line0", "Line1", "Line2", "Software", "FrequencyConverter" 等
    //    请根据您的物理接线选择正确的一项
    ret = MV_CC_SetEnumValueByString(handle_, "TriggerSource", "Line0");
    if (MV_OK != ret)
    {
      RCLCPP_ERROR(this->get_logger(), "Set Trigger Source to Line0 fail! 0x%X", ret);
      return;
    }

    //    (可选) 设置触发激活方式
    //    例如设置为上升沿触发 "RisingEdge"
    //    其他可选值如 "FallingEdge", "LevelHigh", "LevelLow"
    ret = MV_CC_SetEnumValueByString(handle_, "TriggerActivation", "RisingEdge");
    if (MV_OK != ret)
    {
      RCLCPP_ERROR(this->get_logger(), "Set Trigger Activation to RisingEdge fail! 0x%X",
                   ret);
      return;
    }
  }
  else
  {
    // 将触发模式设置为开启 (On)
    // 参数 "TriggerMode" 的值: 0 表示 Off, 1 表示 On
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    if (MV_OK != ret)
    {
      RCLCPP_ERROR(this->get_logger(), "Set Trigger Mode to Off fail! 0x%X", ret);
      return;
    }
  }
  // 曝光、增益、白平衡等
  SetEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  SetEnumValue("GainAuto", MV_GAIN_MODE_OFF);
  SetEnumValue("PixelFormat", PixelType_Gvsp_BayerRG8);
  SetFloatValue("ExposureTime", params_.exposure_time);
  SetFloatValue("Gain", params_.gain);

  MVCC_ENUMVALUE adc_bit_depth{};
  ret = MV_CC_GetEnumValue(handle_, "ADCBitDepth", &adc_bit_depth);
  if (ret == MV_OK)
  {
    RCLCPP_INFO(this->get_logger(), "Current ADCBitDepth: %u", adc_bit_depth.nCurValue);
    // 设置 ADC 位深为 8 Bits (对应枚举值 2)
    ret = MV_CC_SetEnumValue(handle_, "ADCBitDepth", 2);
    if (MV_OK != ret)
    {
      RCLCPP_ERROR(this->get_logger(), "Set ADC Bit Depth to 8 Bits fail! 0x%X", ret);
      return;
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Get ADCBitDepth failed: 0x%X, skip", ret);
  }

  ret = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable",
                           params_.frame_rate_enable);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "MV_CC_SetBoolValue(set frame rate enable) failed: 0x%X", ret);
    return;
  }
  if (params_.frame_rate_enable)
  {
    // 帧率
    ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
                              static_cast<float>(params_.frame_rate));
    if (ret != MV_OK)
    {
      RCLCPP_ERROR(this->get_logger(), "MV_CC_SetFloatValue(set framerate) failed: 0x%X",
                   ret);
      return;
    }
  }
  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_StartGrabbing failed: 0x%X", ret);
    return;
  }

  hik_state_.store(HikStateEnum::RUNNING);
  RCLCPP_INFO(this->get_logger(), "Hik camera initialized and started.");
  return;
}

void HikCameraNode::CaptureStop()
{
  hik_state_.store(HikStateEnum::STOPPED);

  if (handle_ == nullptr)
  {
    return;
  }

  unsigned int ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_StopGrabbing failed: 0x%X", ret);
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_CloseDevice failed: 0x%X", ret);
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_DestroyHandle failed: 0x%X", ret);
  }

  handle_ = nullptr;
  RCLCPP_INFO(this->get_logger(), "Hik camera stopped and handle destroyed.");
}

void HikCameraNode::ProtectRunning()
{
  RCLCPP_INFO(this->get_logger(), "Protect thread started.");

  std::unique_lock<std::mutex> lock(this->guard_.mux);
  while (running_.load())
  {
    // 等待条件变量
    this->guard_.is_quit.wait(
        lock,
        [this]
        {
          return (this->hik_state_.load() == HikStateEnum::STOPPED &&
                  !this->is_switching_.load()) ||
                 (!this->running_.load());
        });

    if (!this->running_.load())
    {
      break;
    }

    RCLCPP_INFO(this->get_logger(), "Camera stopped, attempting to restart...");
    this->CaptureStop();
    // 简单延时防止频繁重启
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    this->CaptureInit();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(this->get_logger(), "Protect thread exit.");
}

void HikCameraNode::SwitchCamera(bool to_lob)
{
  is_switching_.store(true, std::memory_order_seq_cst);
  RCLCPP_INFO(this->get_logger(), "Switching to %s camera...", to_lob ? "lob" : "normal");

  hik_state_.store(HikStateEnum::STOPPED, std::memory_order_seq_cst);

  // 自旋等待in_read_清零
  while (in_read_.load(std::memory_order_seq_cst))
  {
    std::this_thread::yield();
  }

  CaptureStop();

  current_device_index_ = to_lob ? params_.device_index_lob : params_.device_index;

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

  if (hik_state_.load() == HikStateEnum::RUNNING)
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

void HikCameraNode::SetFloatValue(const std::string& name, double value)
{
  if (handle_ == nullptr)
  {
    return;
  }

  unsigned int ret =
      MV_CC_SetFloatValue(handle_, name.c_str(), static_cast<float>(value));
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_SetFloatValue(\"%s\", %f) failed: 0x%X",
                 name.c_str(), value, ret);
  }
}

void HikCameraNode::SetEnumValue(const std::string& name, unsigned int value)
{
  if (handle_ == nullptr)
  {
    return;
  }

  unsigned int ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
  if (ret != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_SetEnumValue(\"%s\", %u) failed: 0x%X",
                 name.c_str(), value, ret);
  }
}
std::string HikCameraNode::PhaseError(unsigned int error_code)
{
  static const std::unordered_map<unsigned int, std::string> ERROR_MAP = {
      // 正确码
      {MV_OK, "成功，无错误"},
      // 通用错误码 (0x80000000-0x800000FF)
      {MV_E_HANDLE, "错误或无效的句柄"},
      {MV_E_SUPPORT, "不支持的功能"},
      {MV_E_BUFOVER, "缓存已满"},
      {MV_E_CALLORDER, "函数调用顺序错误"},
      {MV_E_PARAMETER, "错误的参数"},
      {MV_E_RESOURCE, "资源申请失败"},
      {MV_E_NODATA, "无数据"},
      {MV_E_PRECONDITION, "前置条件有误，或运行环境已发生变化"},
      {MV_E_VERSION, "版本不匹配"},
      {MV_E_NOENOUGH_BUF, "传入的内存空间不足"},
      {MV_E_ABNORMAL_IMAGE, "异常图像，可能是丢包导致图像不完整"},
      {MV_E_LOAD_LIBRARY, "动态导入DLL失败"},
      {MV_E_NOOUTBUF, "没有可输出的缓存"},
      {MV_E_UNKNOW, "未知的错误"},
      // GenICam 系列错误 (0x80000100-0x800001FF)
      {MV_E_GC_GENERIC, "通用错误"},
      {MV_E_GC_ARGUMENT, "参数非法"},
      {MV_E_GC_RANGE, "值超出范围"},
      {MV_E_GC_PROPERTY, "属性错误"},
      {MV_E_GC_RUNTIME, "运行环境有问题"},
      {MV_E_GC_LOGICAL, "逻辑错误"},
      {MV_E_GC_ACCESS, "节点访问条件有误"},
      {MV_E_GC_TIMEOUT, "超时"},
      {MV_E_GC_DYNAMICCAST, "转换异常"},
      {MV_E_GC_UNKNOW, "GenICam未知错误"},
      // GigE_STATUS 错误码 (0x80000200-0x800002FF)
      {MV_E_NOT_IMPLEMENTED, "命令不被设备支持"},
      {MV_E_INVALID_ADDRESS, "访问的目标地址不存在"},
      {MV_E_WRITE_PROTECT, "目标地址不可写"},
      {MV_E_ACCESS_DENIED, "设备无访问权限"},
      {MV_E_BUSY, "设备忙，或网络断开"},
      {MV_E_PACKET, "网络包数据错误"},
      {MV_E_NETER, "网络相关错误"},
      {MV_E_IP_CONFLICT, "设备IP冲突"},
      // USB_STATUS 错误码 (0x80000300-0x800003FF)
      {MV_E_USB_READ, "读usb出错"},
      {MV_E_USB_WRITE, "写usb出错"},
      {MV_E_USB_DEVICE, "设备异常"},
      {MV_E_USB_GENICAM, "GenICam相关错误"},
      {MV_E_USB_BANDWIDTH, "带宽不足"},
      {MV_E_USB_DRIVER, "驱动不匹配或者未装驱动"},
      {MV_E_USB_UNKNOW, "USB未知的错误"},
      // 升级错误码 (0x80000400-0x800004FF)
      {MV_E_UPG_FILE_MISMATCH, "升级固件不匹配"},
      {MV_E_UPG_LANGUSGE_MISMATCH, "升级固件语言不匹配"},
      {MV_E_UPG_CONFLICT, "升级冲突"},
      {MV_E_UPG_INNER_ERR, "相机内部出现错误"},
      {MV_E_UPG_UNKNOW, "升级时未知错误"},
      // ISP 算法库错误码 (0x10000000+)
      {MV_ALG_OK, "ISP: 处理正确"},
      {MV_ALG_ERR, "ISP: 不确定类型错误"},
      {MV_ALG_E_ABILITY_ARG, "ISP: 能力集中存在无效参数"},
      {MV_ALG_E_MEM_NULL, "ISP: 内存地址为空"},
      {MV_ALG_E_MEM_ALIGN, "ISP: 内存对齐不满足要求"},
      {MV_ALG_E_MEM_LACK, "ISP: 内存空间大小不够"},
      {MV_ALG_E_MEM_SIZE_ALIGN, "ISP: 内存空间大小不满足对齐要求"},
      {MV_ALG_E_MEM_ADDR_ALIGN, "ISP: 内存地址不满足对齐要求"},
      {MV_ALG_E_IMG_FORMAT, "ISP: 图像格式不正确或者不支持"},
      {MV_ALG_E_IMG_SIZE, "ISP: 图像宽高不正确或者超出范围"},
      {MV_ALG_E_IMG_STEP, "ISP: 图像宽高与step参数不匹配"},
      {MV_ALG_E_IMG_DATA_NULL, "ISP: 图像数据存储地址为空"},
      {MV_ALG_E_CFG_TYPE, "ISP: 参数类型不正确"},
      {MV_ALG_E_CFG_SIZE, "ISP: 参数结构体大小不正确"},
      {MV_ALG_E_PRC_TYPE, "ISP: 处理类型不正确"},
      {MV_ALG_E_PRC_SIZE, "ISP: 处理参数大小不正确"},
      {MV_ALG_E_FUNC_TYPE, "ISP: 子处理类型不正确"},
      {MV_ALG_E_FUNC_SIZE, "ISP: 子处理参数大小不正确"},
      {MV_ALG_E_PARAM_INDEX, "ISP: index参数不正确"},
      {MV_ALG_E_PARAM_VALUE, "ISP: value参数不正确或者超出范围"},
      {MV_ALG_E_PARAM_NUM, "ISP: param_num参数不正确"},
      {MV_ALG_E_NULL_PTR, "ISP: 函数参数指针为空"},
      {MV_ALG_E_OVER_MAX_MEM, "ISP: 超过限定的最大内存"},
      {MV_ALG_E_CALL_BACK, "ISP: 回调函数出错"},
      {MV_ALG_E_ENCRYPT, "ISP: 加密错误"},
      {MV_ALG_E_EXPIRE, "ISP: 算法库使用期限错误"},
      {MV_ALG_E_BAD_ARG, "ISP: 参数范围不正确"},
      {MV_ALG_E_DATA_SIZE, "ISP: 数据大小不正确"},
      {MV_ALG_E_STEP, "ISP: 数据step不正确"},
      {MV_ALG_E_CPUID, "ISP: cpu不支持优化代码中的指令集"},
      {MV_ALG_WARNING, "ISP: 警告"},
      {MV_ALG_E_TIME_OUT, "ISP: 算法库超时"},
      {MV_ALG_E_LIB_VERSION, "ISP: 算法版本号出错"},
      {MV_ALG_E_MODEL_VERSION, "ISP: 模型版本号出错"},
      {MV_ALG_E_GPU_MEM_ALLOC, "ISP: GPU内存分配错误"},
      {MV_ALG_E_FILE_NON_EXIST, "ISP: 文件不存在"},
      {MV_ALG_E_NONE_STRING, "ISP: 字符串为空"},
      {MV_ALG_E_IMAGE_CODEC, "ISP: 图像解码器错误"},
      {MV_ALG_E_FILE_OPEN, "ISP: 打开文件错误"},
      {MV_ALG_E_FILE_READ, "ISP: 文件读取错误"},
      {MV_ALG_E_FILE_WRITE, "ISP: 文件写错误"},
      {MV_ALG_E_FILE_READ_SIZE, "ISP: 文件读取大小错误"},
      {MV_ALG_E_FILE_TYPE, "ISP: 文件类型错误"},
      {MV_ALG_E_MODEL_TYPE, "ISP: 模型类型错误"},
      {MV_ALG_E_MALLOC_MEM, "ISP: 分配内存错误"},
      {MV_ALG_E_BIND_CORE_FAILED, "ISP: 线程绑核失败"},
      // 降噪特有错误码
      {MV_ALG_E_DENOISE_NE_IMG_FORMAT, "ISP: 噪声特性图像格式错误"},
      {MV_ALG_E_DENOISE_NE_FEATURE_TYPE, "ISP: 噪声特性类型错误"},
      {MV_ALG_E_DENOISE_NE_PROFILE_NUM, "ISP: 噪声特性个数错误"},
      {MV_ALG_E_DENOISE_NE_GAIN_NUM, "ISP: 噪声特性增益个数错误"},
      {MV_ALG_E_DENOISE_NE_GAIN_VAL, "ISP: 噪声曲线增益值输入错误"},
      {MV_ALG_E_DENOISE_NE_BIN_NUM, "ISP: 噪声曲线柱数错误"},
      {MV_ALG_E_DENOISE_NE_INIT_GAIN, "ISP: 噪声估计初始化增益设置错误"},
      {MV_ALG_E_DENOISE_NE_NOT_INIT, "ISP: 噪声估计未初始化"},
      {MV_ALG_E_DENOISE_COLOR_MODE, "ISP: 颜色空间模式错误"},
      {MV_ALG_E_DENOISE_ROI_NUM, "ISP: 图像ROI个数错误"},
      {MV_ALG_E_DENOISE_ROI_ORI_PT, "ISP: 图像ROI原点错误"},
      {MV_ALG_E_DENOISE_ROI_SIZE, "ISP: 图像ROI大小错误"},
      {MV_ALG_E_DENOISE_GAIN_NOT_EXIST, "ISP: 输入的相机增益不存在"},
      {MV_ALG_E_DENOISE_GAIN_BEYOND_RANGE, "ISP: 输入的相机增益不在范围内"},
      {MV_ALG_E_DENOISE_NP_BUF_SIZE, "ISP: 输入的噪声特性内存大小错误"},
  };

  auto it = ERROR_MAP.find(error_code);
  if (it != ERROR_MAP.end())
  {
    return it->second;
  }

  return "Unknown error code: 0x" + std::string(error_code & 0x80000000 ? "" : "-") +
         std::to_string(error_code);
}
}  // namespace HikCamera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(HikCamera::HikCameraNode)
