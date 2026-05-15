#ifndef HAND_EYE_CALIBRATE_NODE_HPP_
#define HAND_EYE_CALIBRATE_NODE_HPP_

// ======================== ROS2 ========================
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

// ======================== LibXR ========================
#include "SharedTopic.hpp"
#include "linux_uart.hpp"

static void XRobotMain(LibXR::HardwareContainer& hw)
{
  using namespace LibXR;
  static ApplicationManager appmgr;

  // 只从下位机接收 ahrs_quaternion，不发送任何数据
  static SharedTopic shared_topic(hw, appmgr, "uart_client", 256, {{"ahrs_quaternion"}});
}

// =====================================================
// HandEyeCalibrateNode
// =====================================================
class HandEyeCalibrateNode : public rclcpp::Node
{
 public:
  explicit HandEyeCalibrateNode(const rclcpp::NodeOptions& options);
  ~HandEyeCalibrateNode() override = default;

 private:
  // -------------------- 数据结构 --------------------
  struct Detection
  {
    rclcpp::Time stamp;
    cv::Mat R_target2cam;               // 3x3  PnP: 标定板→相机
    cv::Mat t_target2cam;               // 3x1
    cv::Mat R_gimbal2world;             // 3x3  用于 calibrateHandEye
    cv::Mat R_world2gimbal;             // 3x3  用于 calibrateRobotWorldHandEye
    cv::Mat t_gimbal;                   // 3x1  全零（云台纯旋转）
    Eigen::Quaterniond q_gimbal2world;  // 用于去重
    double reproj_rmse = 0.0;
    double blur_score = 0.0;
  };

  struct VisInfo
  {
    bool has_board = false;
    double blur = 0.0;
    bool is_static = true;
    double rmse = 0.0;
  };

  // -------------------- 辅助函数 --------------------
  static double LimitRad(double angle);
  static Eigen::Vector3d QuatToEulers(const Eigen::Quaterniond& q, int axis0, int axis1,
                                      int axis2);
  static Eigen::Vector3d MatToEulers(const Eigen::Matrix3d& R, int a0, int a1, int a2);
  static double CalcBlurScore(const cv::Mat& gray);

  bool CheckStatic(const Eigen::Quaterniond& q_current) const;
  Eigen::Matrix3d ImuQuatToGimbalWorld(const Eigen::Quaterniond& q_imu) const;
  int ToOpenCvMethod(const std::string& m) const;

  // -------------------- ROS2 回调 --------------------
  void CameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void ImageCallback(sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void OnCapture(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void OnReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void OnSolve(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // -------------------- 求解 --------------------
  void SolveClassic(const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void SolveRobotWorld(const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void FormatResult(const cv::Mat& R_cam2gimbal, const cv::Mat& t_cam2gimbal,
                    const std::string& method_name,
                    const std::shared_ptr<std_srvs::srv::Trigger::Response> res,
                    const std::string& extra_info = "");

  // ==================== LibXR 串口设施 ====================
  std::unique_ptr<LibXR::RamFS> ramfs_;
  std::unique_ptr<LibXR::LinuxUART> uart_client_;
  std::unique_ptr<LibXR::Terminal<1024, 64, 16, 128>> terminal_;
  std::unique_ptr<LibXR::Thread> term_thread_;
  std::unique_ptr<LibXR::HardwareContainer> peripherals_;
  LibXR::Topic ahrs_quaternion_topic_;  // 接收下位机四元数
  double timestamp_offset_ = 0.0;

  // ==================== 标定参数 ====================
  std::string image_topic_, camera_info_topic_;
  int board_cols_, board_rows_;
  double square_size_;
  std::vector<double> r_gimbal2imubody_data_;
  Eigen::Matrix3d r_gimbal2imubody_;
  std::string method_;
  bool use_robot_world_;
  double min_blur_score_, min_angle_dist_rad_, max_age_sec_;
  double max_quat_angular_vel_;
  bool publish_debug_image_;
  std::string debug_image_topic_;

  // ==================== 运行时状态 ====================
  std::mutex mtx_;
  std::mutex quat_mtx_;
  bool have_intrinsics_ = false;
  bool have_quat_ = false;
  cv::Mat k_, d_;
  Eigen::Quaterniond latest_quat_{1, 0, 0, 0};
  rclcpp::Time latest_quat_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<Eigen::Quaterniond> prev_quat_;
  std::optional<Detection> last_detection_;
  VisInfo current_vis_{};
  std::vector<Detection> samples_;
  int ahrs_receive_cnt_ = 0;

  // ==================== ROS2 接口 ====================
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_, reset_srv_, solve_srv_;
};

#endif  // HAND_EYE_CALIBRATE_NODE_HPP_
