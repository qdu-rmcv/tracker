#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

class VideoPublisherNode : public rclcpp::Node
{
 public:
  explicit VideoPublisherNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("video_publisher", options)
  {
    video_path_ = this->declare_parameter<std::string>("video_path", "");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
    camera_name_ = this->declare_parameter<std::string>("camera_name", "gimbal_camera");
    camera_info_url_ = this->declare_parameter<std::string>(
        "camera_info_url", "package://video_publisher/config/camera_info.yaml");
    loop_ = this->declare_parameter<bool>("loop", true);
    use_video_fps_ = this->declare_parameter<bool>("use_video_fps", true);
    playback_rate_ = this->declare_parameter<double>("playback_rate", 1.0);
    publish_fps_ = this->declare_parameter<double>("publish_fps", 30.0);

    save_result_video_ = this->declare_parameter<bool>("save_result_video", false);
    result_img_topic_ =
        this->declare_parameter<std::string>("result_img_topic", "/result_img");
    result_img_transport_ =
        this->declare_parameter<std::string>("result_img_transport", "raw");
    result_video_path_ = this->declare_parameter<std::string>("result_video_path", "");
    // 编码 FourCC，4 个字符：mp4v / avc1 / XVID / MJPG 等
    result_video_fourcc_ =
        this->declare_parameter<std::string>("result_video_fourcc", "mp4v");
    result_video_fps_ = this->declare_parameter<double>("result_video_fps", 0.0);
    result_video_auto_size_ =
        this->declare_parameter<bool>("result_video_auto_size", true);
    result_video_width_ = this->declare_parameter<int>("result_video_width", 0);
    result_video_height_ = this->declare_parameter<int>("result_video_height", 0);
    // 每累计多少帧打印一次状态日志
    result_log_every_n_frames_ =
        this->declare_parameter<int>("result_log_every_n_frames", 100);

    if (video_path_.empty())
    {
      throw std::runtime_error("Parameter 'video_path' is empty.");
    }

    if (playback_rate_ <= 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid playback_rate %.3f, fallback to 1.0.",
                  playback_rate_);
      playback_rate_ = 1.0;
    }

    if (publish_fps_ < 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid publish_fps %.3f, fallback to auto mode.",
                  publish_fps_);
      publish_fps_ = 0.0;
    }

    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw",
                                                           rmw_qos_profile_sensor_data);

    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);

    LoadCameraInfo();
    OpenVideo(video_path_);

    source_timeline_fps_ = GetSourceTimelineFps();
    output_publish_fps_ = GetOutputPublishFps();

    if (!std::isfinite(source_timeline_fps_) || source_timeline_fps_ <= 1e-6)
    {
      throw std::runtime_error("Invalid source timeline fps.");
    }

    if (!std::isfinite(output_publish_fps_) || output_publish_fps_ <= 1e-6)
    {
      throw std::runtime_error("Invalid output publish fps.");
    }

    playback_start_time_ = std::chrono::steady_clock::now();
    last_published_timeline_frame_index_ = -1;
    last_published_frame_index_ = -1;

    auto period = std::chrono::duration<double>(1.0 / output_publish_fps_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&VideoPublisherNode::PublishFrame, this));

    RCLCPP_INFO(this->get_logger(),
                "Video publisher started. path=%s, source_video_fps=%.3f, "
                "timeline_fps=%.3f, publish_fps=%.3f, playback_rate=%.3f, "
                "total_frames=%lld, loop=%s",
                video_path_.c_str(), video_fps_, source_timeline_fps_,
                output_publish_fps_, playback_rate_,
                static_cast<long long>(total_frames_), loop_ ? "true" : "false");

    SetupResultImageSubscriber();
  }

  ~VideoPublisherNode() override { CloseResultVideoWriter(); }

 private:
  void LoadCameraInfo()
  {
    if (!camera_info_manager_->validateURL(camera_info_url_))
    {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                  camera_info_url_.c_str());
      camera_info_msg_.header.frame_id = frame_id_;
      return;
    }

    if (!camera_info_manager_->loadCameraInfo(camera_info_url_))
    {
      RCLCPP_WARN(this->get_logger(), "Failed to load camera info from: %s",
                  camera_info_url_.c_str());
      camera_info_msg_.header.frame_id = frame_id_;
      return;
    }

    camera_info_msg_ = camera_info_manager_->getCameraInfo();
    camera_info_msg_.header.frame_id = frame_id_;
    RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s",
                camera_info_url_.c_str());
  }

  void OpenVideo(const std::string& video_path)
  {
    cap_.release();
    if (!cap_.open(video_path))
    {
      throw std::runtime_error("Failed to open video file: " + video_path);
    }

    double detected_fps = cap_.get(cv::CAP_PROP_FPS);
    video_fps_ = std::isfinite(detected_fps) && detected_fps > 1e-6 ? detected_fps : 0.0;

    double detected_total_frames = cap_.get(cv::CAP_PROP_FRAME_COUNT);
    if (std::isfinite(detected_total_frames) && detected_total_frames > 0.0)
    {
      total_frames_ = static_cast<int64_t>(std::llround(detected_total_frames));
    }
    else
    {
      total_frames_ = 0;
    }

    RCLCPP_INFO(this->get_logger(),
                "Opened video: %s (video_fps=%.3f, total_frames=%lld)",
                video_path.c_str(), video_fps_, static_cast<long long>(total_frames_));
  }

  double GetSourceTimelineFps() const
  {
    double base_fps = 0.0;

    if (video_fps_ > 1e-6)
    {
      base_fps = video_fps_;
    }
    else if (publish_fps_ > 1e-6)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Video metadata fps invalid, fallback to publish_fps %.3f "
                  "for timeline progression.",
                  publish_fps_);
      base_fps = publish_fps_;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "Video metadata fps invalid and publish_fps is 0. "
                  "Fallback to 30.0 fps for timeline progression.");
      base_fps = 30.0;
    }

    return base_fps * playback_rate_;
  }

  double GetOutputPublishFps() const
  {
    double fps = 0.0;

    if (publish_fps_ > 1e-6)
    {
      fps = publish_fps_;
    }
    else if (use_video_fps_ && video_fps_ > 1e-6)
    {
      fps = video_fps_;
    }
    else
    {
      fps = 30.0;
    }

    if (!std::isfinite(fps) || fps <= 1e-6)
    {
      fps = 30.0;
    }

    return fps;
  }

  int64_t ComputeTargetFrameIndex() const
  {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - playback_start_time_;
    double elapsed_seconds = elapsed.count();

    if (elapsed_seconds < 0.0)
    {
      return 0;
    }

    double target_frame = std::floor(elapsed_seconds * source_timeline_fps_);
    if (!std::isfinite(target_frame) || target_frame < 0.0)
    {
      return 0;
    }

    return static_cast<int64_t>(target_frame);
  }

  bool SeekAndReadFrame(int64_t frame_index, cv::Mat& frame)
  {
    if (frame_index < 0)
    {
      return false;
    }

    if (!cap_.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frame_index)))
    {
      RCLCPP_WARN(this->get_logger(), "Failed to seek to frame %lld.",
                  static_cast<long long>(frame_index));
      return false;
    }

    if (!cap_.read(frame) || frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Failed to read frame %lld.",
                  static_cast<long long>(frame_index));
      return false;
    }

    return true;
  }

  bool ReadFrameAt(int64_t frame_index, cv::Mat& frame)
  {
    if (frame_index < 0)
    {
      return false;
    }

    if (last_published_frame_index_ >= 0 &&
        frame_index == last_published_frame_index_ + 1)
    {
      if (cap_.read(frame) && !frame.empty())
      {
        return true;
      }

      RCLCPP_WARN(this->get_logger(), "Failed to read next sequential frame %lld.",
                  static_cast<long long>(frame_index));
      return false;
    }

    return SeekAndReadFrame(frame_index, frame);
  }

  bool ReopenVideoCapture()
  {
    try
    {
      OpenVideo(video_path_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to reopen video '%s': %s",
                   video_path_.c_str(), e.what());
      return false;
    }

    return true;
  }

  bool ReadFirstFrameForLoop(cv::Mat& frame)
  {
    if (cap_.set(cv::CAP_PROP_POS_FRAMES, 0.0) && cap_.read(frame) && !frame.empty())
    {
      return true;
    }

    RCLCPP_WARN(this->get_logger(),
                "Failed to rewind the current capture, trying to reopen video.");

    if (!ReopenVideoCapture())
    {
      return false;
    }

    if (!cap_.set(cv::CAP_PROP_POS_FRAMES, 0.0))
    {
      RCLCPP_WARN(this->get_logger(), "Failed to seek reopened video to frame 0.");
    }

    if (!cap_.read(frame) || frame.empty())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to read the first frame after reopening video.");
      return false;
    }

    return true;
  }

  void ResetPlaybackClockForLoop()
  {
    playback_start_time_ = std::chrono::steady_clock::now();
    last_published_timeline_frame_index_ = -1;
    last_published_frame_index_ = -1;
  }

  bool ResolveFrameIndex(int64_t& target_frame_index)
  {
    if (total_frames_ <= 0)
    {
      return true;
    }

    if (!loop_)
    {
      if (target_frame_index >= total_frames_)
      {
        return false;
      }
      return true;
    }

    target_frame_index %= total_frames_;
    if (target_frame_index < 0)
    {
      target_frame_index += total_frames_;
    }

    return true;
  }

  void PublishFrame()
  {
    int64_t timeline_frame_index = ComputeTargetFrameIndex();

    if (timeline_frame_index == last_published_timeline_frame_index_)
    {
      return;
    }

    int64_t target_frame_index = timeline_frame_index;
    if (!ResolveFrameIndex(target_frame_index))
    {
      RCLCPP_INFO(this->get_logger(),
                  "Reached end of video timeline, stop publishing video frames.");
      timer_->cancel();
      return;
    }

    if (last_published_timeline_frame_index_ >= 0 &&
        timeline_frame_index > last_published_timeline_frame_index_ + 1)
    {
      RCLCPP_DEBUG(this->get_logger(), "Skipping timeline frames from %lld to %lld.",
                   static_cast<long long>(last_published_timeline_frame_index_ + 1),
                   static_cast<long long>(timeline_frame_index - 1));
    }

    cv::Mat bgr_frame;
    if (!ReadFrameAt(target_frame_index, bgr_frame))
    {
      if (loop_)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Unable to read frame %lld while loop=true; restarting from frame 0.",
                    static_cast<long long>(target_frame_index));

        ResetPlaybackClockForLoop();
        timeline_frame_index = 0;
        target_frame_index = 0;

        if (!ReadFirstFrameForLoop(bgr_frame))
        {
          RCLCPP_WARN(this->get_logger(),
                      "Unable to restart video loop, stop publishing.");
          timer_->cancel();
          return;
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unable to read target frame, stop publishing.");
        timer_->cancel();
        return;
      }
    }

    cv::Mat rgb_frame;
    cv::cvtColor(bgr_frame, rgb_frame, cv::COLOR_BGR2RGB);

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = frame_id_;

    auto image_msg = cv_bridge::CvImage(header, "rgb8", rgb_frame).toImageMsg();

    camera_info_msg_.header = header;
    camera_info_msg_.width = static_cast<uint32_t>(rgb_frame.cols);
    camera_info_msg_.height = static_cast<uint32_t>(rgb_frame.rows);

    camera_pub_.publish(*image_msg, camera_info_msg_);

    last_published_timeline_frame_index_ = timeline_frame_index;
    last_published_frame_index_ = target_frame_index;

    ++published_frame_count_;
    if (published_frame_count_ % 100 == 0)
    {
      auto now = std::chrono::steady_clock::now();
      if (has_publish_stat_)
      {
        std::chrono::duration<double> dt = now - publish_stat_time_;
        if (dt.count() > 1e-6)
        {
          RCLCPP_INFO(this->get_logger(), "Actual publish fps: %.2f", 100.0 / dt.count());
        }
      }
      publish_stat_time_ = now;
      has_publish_stat_ = true;
    }
  }

  void SetupResultImageSubscriber()
  {
    if (!save_result_video_)
    {
      RCLCPP_INFO(this->get_logger(),
                  "save_result_video=false, skipping result image recording.");
      return;
    }

    if (result_video_path_.empty())
    {
      RCLCPP_WARN(this->get_logger(),
                  "save_result_video=true but result_video_path is empty, "
                  "recording disabled.");
      return;
    }

    try
    {
      std::filesystem::path out_path(result_video_path_);
      if (out_path.has_parent_path())
      {
        std::filesystem::create_directories(out_path.parent_path());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to create directory for result video '%s': %s",
                  result_video_path_.c_str(), e.what());
    }

    result_img_sub_ = image_transport::create_subscription(
        this, result_img_topic_,
        std::bind(&VideoPublisherNode::ResultImageCallback, this, std::placeholders::_1),
        result_img_transport_, rmw_qos_profile_sensor_data);

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to result image: topic='%s', transport='%s', "
                "output='%s', fourcc='%s'.",
                result_img_topic_.c_str(), result_img_transport_.c_str(),
                result_video_path_.c_str(), result_video_fourcc_.c_str());
  }

  void ResultImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    if (!msg)
    {
      return;
    }

    cv::Mat frame;
    try
    {
      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      frame = cv_ptr->image;
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in ResultImageCallback: %s",
                   e.what());
      return;
    }

    if (frame.empty())
    {
      return;
    }

    std::lock_guard<std::mutex> lock(result_writer_mutex_);

    if (!result_writer_.isOpened())
    {
      if (!OpenResultVideoWriter(frame.size()))
      {
        return;
      }
    }

    cv::Mat frame_to_write;
    if (frame.size() != result_writer_size_)
    {
      cv::resize(frame, frame_to_write, result_writer_size_);
    }
    else
    {
      frame_to_write = frame;
    }

    result_writer_.write(frame_to_write);
    ++result_saved_count_;

    if (result_log_every_n_frames_ > 0 &&
        static_cast<int>(result_saved_count_ % result_log_every_n_frames_) == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Saved %lu result frames to '%s'.",
                  static_cast<unsigned long>(result_saved_count_),
                  result_video_path_.c_str());
    }
  }

  bool OpenResultVideoWriter(const cv::Size& incoming_size)
  {
    cv::Size target_size;
    if (result_video_auto_size_ || result_video_width_ <= 0 || result_video_height_ <= 0)
    {
      target_size = incoming_size;
    }
    else
    {
      target_size = cv::Size(result_video_width_, result_video_height_);
    }

    double writer_fps = result_video_fps_;
    if (!std::isfinite(writer_fps) || writer_fps <= 1e-6)
    {
      writer_fps = output_publish_fps_;
    }
    if (!std::isfinite(writer_fps) || writer_fps <= 1e-6)
    {
      writer_fps = 30.0;
    }

    if (result_video_fourcc_.size() != 4)
    {
      RCLCPP_WARN(this->get_logger(),
                  "result_video_fourcc must be 4 chars, got '%s'. Fallback to 'mp4v'.",
                  result_video_fourcc_.c_str());
      result_video_fourcc_ = "mp4v";
    }

    int fourcc =
        cv::VideoWriter::fourcc(result_video_fourcc_[0], result_video_fourcc_[1],
                                result_video_fourcc_[2], result_video_fourcc_[3]);

    if (!result_writer_.open(result_video_path_, fourcc, writer_fps, target_size, true))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open VideoWriter '%s' (fourcc=%s, fps=%.3f, size=%dx%d).",
                   result_video_path_.c_str(), result_video_fourcc_.c_str(), writer_fps,
                   target_size.width, target_size.height);
      return false;
    }

    result_writer_size_ = target_size;
    result_writer_fps_ = writer_fps;
    RCLCPP_INFO(this->get_logger(),
                "Opened result VideoWriter: '%s' (fourcc=%s, fps=%.3f, size=%dx%d).",
                result_video_path_.c_str(), result_video_fourcc_.c_str(), writer_fps,
                target_size.width, target_size.height);
    return true;
  }

  void CloseResultVideoWriter()
  {
    std::lock_guard<std::mutex> lock(result_writer_mutex_);
    if (result_writer_.isOpened())
    {
      result_writer_.release();
      RCLCPP_INFO(this->get_logger(),
                  "Closed result VideoWriter. Total frames saved: %lu, path: %s",
                  static_cast<unsigned long>(result_saved_count_),
                  result_video_path_.c_str());
    }
  }

  std::string video_path_;
  std::string frame_id_;
  std::string camera_name_;
  std::string camera_info_url_;
  bool loop_{true};
  bool use_video_fps_{true};
  double playback_rate_{1.0};
  double publish_fps_{30.0};
  double video_fps_{0.0};
  double source_timeline_fps_{30.0};
  double output_publish_fps_{30.0};
  int64_t total_frames_{0};
  int64_t last_published_timeline_frame_index_{-1};
  int64_t last_published_frame_index_{-1};

  cv::VideoCapture cap_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point playback_start_time_;
  std::chrono::steady_clock::time_point publish_stat_time_;
  bool has_publish_stat_{false};
  uint64_t published_frame_count_{0};

  bool save_result_video_{false};
  std::string result_img_topic_;
  std::string result_img_transport_;
  std::string result_video_path_;
  std::string result_video_fourcc_;
  double result_video_fps_{0.0};
  bool result_video_auto_size_{true};
  int result_video_width_{0};
  int result_video_height_{0};
  int result_log_every_n_frames_{100};

  image_transport::Subscriber result_img_sub_;
  cv::VideoWriter result_writer_;
  cv::Size result_writer_size_{0, 0};
  double result_writer_fps_{0.0};
  std::mutex result_writer_mutex_;
  uint64_t result_saved_count_{0};
};

RCLCPP_COMPONENTS_REGISTER_NODE(VideoPublisherNode)
