#include "rm_hand_eye_calibrate/hand_eye_calibrate_node.hpp"

HandEyeCalibrateNode::HandEyeCalibrateNode(const rclcpp::NodeOptions& options)
    : Node("hand_eye_calibrator_node", options)
{
  LibXR::PlatformInit();
  peripherals_ = std::make_unique<LibXR::HardwareContainer>();
  ramfs_ = std::make_unique<LibXR::RamFS>();

  auto vid = this->declare_parameter<std::string>("vid", "16d0");
  auto pid = this->declare_parameter<std::string>("pid", "1492");
  timestamp_offset_ = this->declare_parameter<double>("timestamp_offset", 0.0);

  uart_client_ = std::make_unique<LibXR::LinuxUART>(
      vid, pid, 115200, LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);
  terminal_ = std::make_unique<LibXR::Terminal<1024, 64, 16, 128>>(*ramfs_);
  term_thread_ = std::make_unique<LibXR::Thread>();
  term_thread_->Create(terminal_.get(), LibXR::Terminal<1024, 64, 16, 128>::ThreadFun,
                       "terminal", 81900, LibXR::Thread::Priority::MEDIUM);

  static LibXR::HardwareContainer peripherals{
      LibXR::Entry<LibXR::RamFS>({*ramfs_, {"ramfs"}}),
      LibXR::Entry<LibXR::UART>({*uart_client_, {"uart_client"}}),
  };

  ahrs_quaternion_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("ahrs_quaternion");

  XRobotMain(peripherals);

  void (*ahrs_quaternion_cb_fun)(bool, HandEyeCalibrateNode* self, LibXR::RawData& data) =
      [](bool, HandEyeCalibrateNode* self, LibXR::RawData& data)
  {
    auto quat = reinterpret_cast<LibXR::Quaternion<float>*>(data.addr_);
    Eigen::Quaterniond q_eigen(
        static_cast<double>(quat->w()), static_cast<double>(quat->x()),
        static_cast<double>(quat->y()), static_cast<double>(quat->z()));
    q_eigen.normalize();

    rclcpp::Time stamp =
        self->now() + rclcpp::Duration::from_seconds(self->timestamp_offset_);
    std::lock_guard<std::mutex> lk(self->quat_mtx_);
    self->latest_quat_ = q_eigen;
    self->latest_quat_stamp_ = stamp;
    self->have_quat_ = true;

    if (++self->ahrs_receive_cnt_ % 50 == 0)
    {
      Eigen::Matrix3d r_g2w = self->ImuQuatToGimbalWorld(q_eigen);
      Eigen::Vector3d ypr = MatToEulers(r_g2w, 2, 1, 0) * 57.3;
      RCLCPP_DEBUG(self->get_logger(), "Gimbal YPR: [%.2f, %.2f, %.2f] deg", ypr[0],
                   ypr[1], ypr[2]);
      self->ahrs_receive_cnt_ = 0;
    }
  };
  auto ahrs_quaternion_cb = LibXR::Topic::Callback::Create(ahrs_quaternion_cb_fun, this);
  ahrs_quaternion_topic_.RegisterCallback(ahrs_quaternion_cb);

  image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
  camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera_info");

  board_cols_ = static_cast<int>(declare_parameter<int>("board_cols", 11));
  board_rows_ = static_cast<int>(declare_parameter<int>("board_rows", 8));
  square_size_ = declare_parameter<double>("square_size", 0.02);

  auto default_r = std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1};

  r_gimbal2imubody_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      declare_parameter<std::vector<double>>("R_gimbal2imubody", default_r).size() == 9
          ? r_gimbal2imubody_data_.data()
          : default_r.data());

  method_ = declare_parameter<std::string>("handeye_method", "TSAI");
  use_robot_world_ = declare_parameter<bool>("use_robot_world_handeye", true);

  min_blur_score_ = declare_parameter<double>("check_min_blur_score", 60.0);
  min_angle_dist_rad_ = declare_parameter<double>("check_min_angle_dist", 0.087);
  max_age_sec_ = declare_parameter<double>("max_age_sec", 0.5);
  max_quat_angular_vel_ = declare_parameter<double>("check_max_quat_angular_vel", 0.02);

  publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
  debug_image_topic_ =
      declare_parameter<std::string>("debug_image_topic", "/rm_hand_eye/debug_image");

  debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 1);

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HandEyeCalibrateNode::CameraInfoCallback, this, std::placeholders::_1));

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HandEyeCalibrateNode::ImageCallback, this, std::placeholders::_1));

  capture_srv_ = create_service<std_srvs::srv::Trigger>(
      "/rm_hand_eye/capture", std::bind(&HandEyeCalibrateNode::OnCapture, this,
                                        std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/rm_hand_eye/reset", std::bind(&HandEyeCalibrateNode::OnReset, this,
                                      std::placeholders::_1, std::placeholders::_2));
  solve_srv_ = create_service<std_srvs::srv::Trigger>(
      "/rm_hand_eye/solve", std::bind(&HandEyeCalibrateNode::OnSolve, this,
                                      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "HandEye 标定节点已启动 [模式: %s]",
              use_robot_world_ ? "RobotWorld (AX=ZB)" : "Classic (AX=XB)");
}

double HandEyeCalibrateNode::LimitRad(double angle)
{
  while (angle > CV_PI)
  {
    angle -= 2 * CV_PI;
  }
  while (angle < -CV_PI)
  {
    angle += 2 * CV_PI;
  }
  return angle;
}

Eigen::Vector3d HandEyeCalibrateNode::QuatToEulers(const Eigen::Quaterniond& q, int axis0,
                                                   int axis1, int axis2)
{
  bool extrinsic = true;
  int i = axis0, j = axis1, k = axis2;
  bool is_proper = (i == k);
  if (is_proper)
  {
    k = 3 - i - j;
  }
  int sign = (i - j) * (j - k) * (k - i) / 2;

  Eigen::Vector4d xyzw = q.coeffs();
  double a = NAN, b = NAN, c = NAN, d = NAN;
  if (is_proper)
  {
    a = xyzw[3];
    b = xyzw[i];
    c = xyzw[j];
    d = xyzw[k] * sign;
  }
  else
  {
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }

  Eigen::Vector3d eulers;
  double n2 = a * a + b * b + c * c + d * d;
  eulers[1] = std::acos(std::clamp(2.0 * (a * a + b * b) / n2 - 1.0, -1.0, 1.0));

  double half_sum = std::atan2(b, a);
  double half_diff = std::atan2(-d, c);
  double eps = 1e-7;
  bool safe1 = std::abs(eulers[1]) >= eps;
  bool safe2 = std::abs(eulers[1] - CV_PI) >= eps;

  if (safe1 && safe2)
  {
    eulers[0] = half_sum + half_diff;
    eulers[2] = half_sum - half_diff;
  }
  else
  {
    if (extrinsic)
    {
      eulers[2] = 0;
      eulers[0] = safe1 ? 2 * half_sum : 2 * half_diff;
    }
    else
    {
      eulers[0] = 0;
      eulers[2] = safe1 ? 2 * half_sum : -2 * half_diff;
    }
  }

  for (int idx = 0; idx < 3; idx++)
  {
    eulers[idx] = LimitRad(eulers[idx]);
  }
  if (!is_proper)
  {
    eulers[2] *= sign;
    eulers[1] -= CV_PI / 2;
  }
  if (!extrinsic)
  {
    std::swap(eulers[0], eulers[2]);
  }

  return eulers;
}

Eigen::Vector3d HandEyeCalibrateNode::MatToEulers(const Eigen::Matrix3d& R, int a0,
                                                  int a1, int a2)
{
  return QuatToEulers(Eigen::Quaterniond(R), a0, a1, a2);
}

double HandEyeCalibrateNode::CalcBlurScore(const cv::Mat& gray)
{
  cv::Mat lap;
  cv::Laplacian(gray, lap, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(lap, mean, stddev);
  return stddev.val[0] * stddev.val[0];
}

bool HandEyeCalibrateNode::CheckStatic(const Eigen::Quaterniond& q_current) const
{
  if (!prev_quat_.has_value())
  {
    return true;
  }
  return prev_quat_->angularDistance(q_current) < max_quat_angular_vel_;
}

Eigen::Matrix3d HandEyeCalibrateNode::ImuQuatToGimbalWorld(
    const Eigen::Quaterniond& q_imu) const
{
  Eigen::Matrix3d r_imubody2imuabs = q_imu.toRotationMatrix();
  return r_gimbal2imubody_.transpose() * r_imubody2imuabs * r_gimbal2imubody_;
}

int HandEyeCalibrateNode::ToOpenCvMethod(const std::string& m) const
{
  std::string u = m;
  std::transform(u.begin(), u.end(), u.begin(), ::toupper);
  if (u == "TSAI")
  {
    return cv::CALIB_HAND_EYE_TSAI;
  }
  if (u == "PARK")
  {
    return cv::CALIB_HAND_EYE_PARK;
  }
  if (u == "HORAUD")
  {
    return cv::CALIB_HAND_EYE_HORAUD;
  }
  if (u == "ANDREFF")
  {
    return cv::CALIB_HAND_EYE_ANDREFF;
  }
  if (u == "DANIILIDIS")
  {
    return cv::CALIB_HAND_EYE_DANIILIDIS;
  }
  return cv::CALIB_HAND_EYE_TSAI;
}

void HandEyeCalibrateNode::CameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  k_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4],
        msg->k[5], msg->k[6], msg->k[7], msg->k[8]);
  d_ = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
  for (size_t i = 0; i < msg->d.size(); ++i)
  {
    d_.at<double>(static_cast<int>(i), 0) = msg->d[i];
  }
  have_intrinsics_ = true;
}

void HandEyeCalibrateNode::ImageCallback(sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  if (!have_intrinsics_ || !have_quat_)
  {
    return;
  }

  // 取当前四元数快照
  Eigen::Quaterniond q_imu;
  rclcpp::Time q_stamp;
  {
    std::lock_guard<std::mutex> lk(quat_mtx_);
    q_imu = latest_quat_;
    q_stamp = latest_quat_stamp_;
  }

  // 检查四元数时间戳与图像时间戳的偏差
  double dt = std::abs((rclcpp::Time(img_msg->header.stamp) - q_stamp).seconds());
  if (dt > max_age_sec_)
  {
    return;
  }

  // IMU 四元数 → 云台姿态
  Eigen::Matrix3d r_gimbal2world_eigen = ImuQuatToGimbalWorld(q_imu);
  Eigen::Quaterniond q_gimbal2world(r_gimbal2world_eigen);
  Eigen::Vector3d ypr = MatToEulers(r_gimbal2world_eigen, 2, 1, 0) * 57.3;

  bool is_static = CheckStatic(q_gimbal2world);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    prev_quat_ = q_gimbal2world;
  }

  // 解码图像
  cv::Mat frame;
  try
  {
    frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  }
  catch (...)
  {
    return;
  }

  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  double blur = CalcBlurScore(gray);

  // 检测标定板
  const cv::Size PATTERN_SIZE(board_cols_, board_rows_);
  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(
      gray, PATTERN_SIZE, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

  // ---- 可视化 ----
  cv::Mat vis = frame.clone();
  auto draw_text = [&](const std::string& txt, bool ok, int line)
  {
    cv::Scalar color = ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(vis, txt, cv::Point(20, 30 * line), cv::FONT_HERSHEY_SIMPLEX, 0.7, color,
                2);
  };

  // 显示云台欧拉角 —— 验证 R_gimbal2imubody 是否正确
  std::ostringstream yaw_s, pitch_s, roll_s;
  yaw_s << "yaw:   " << std::fixed << std::setprecision(2) << ypr[0];
  pitch_s << "pitch: " << std::fixed << std::setprecision(2) << ypr[1];
  roll_s << "roll:  " << std::fixed << std::setprecision(2) << ypr[2];
  draw_text(yaw_s.str(), true, 1);
  draw_text(pitch_s.str(), true, 2);
  draw_text(roll_s.str(), true, 3);
  draw_text(is_static ? "STATIC" : "MOVING", is_static, 4);

  if (found)
  {
    cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
    cv::drawChessboardCorners(vis, PATTERN_SIZE, corners, true);

    std::vector<cv::Point3f> obj_pts;
    for (int r = 0; r < board_rows_; ++r)
    {
      for (int c = 0; c < board_cols_; ++c)
      {
        obj_pts.emplace_back(static_cast<float>(c * square_size_),
                             static_cast<float>(r * square_size_), 0.0f);
      }
    }

    cv::Mat rvec, tvec;
    cv::solvePnP(obj_pts, corners, k_, d_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    // 重投影误差
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj_pts, rvec, tvec, k_, d_, proj);
    double err_sum = 0;
    for (size_t i = 0; i < corners.size(); ++i)
    {
      err_sum += cv::norm(corners[i] - proj[i]);
    }
    double rmse = std::sqrt(err_sum / static_cast<double>(corners.size()));

    // 组装 Detection
    Detection det;
    det.stamp = img_msg->header.stamp;
    cv::Rodrigues(rvec, det.R_target2cam);
    det.t_target2cam = tvec.clone();
    det.t_gimbal = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
    det.q_gimbal2world = q_gimbal2world;
    det.reproj_rmse = rmse;
    det.blur_score = blur;

    cv::eigen2cv(r_gimbal2world_eigen, det.R_gimbal2world);
    Eigen::Matrix3d r_w2g = r_gimbal2world_eigen.transpose();
    cv::eigen2cv(r_w2g, det.R_world2gimbal);

    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_detection_ = det;
      current_vis_ = {true, blur, is_static, rmse};
    }

    std::ostringstream s_blur, s_rmse;
    s_blur << "Blur: " << std::fixed << std::setprecision(1) << blur;
    s_rmse << "PnP RMSE: " << std::fixed << std::setprecision(3) << rmse;
    draw_text(s_blur.str(), blur >= min_blur_score_, 5);
    draw_text(s_rmse.str(), rmse < 1.0, 6);
    draw_text("Samples: " + std::to_string(samples_.size()), true, 7);
  }
  else
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_detection_.reset();
    current_vis_ = {false, blur, is_static, 0.0};
    draw_text("NO CHESSBOARD", false, 5);
  }

  if (publish_debug_image_ && debug_image_pub_)
  {
    debug_image_pub_->publish(
        *cv_bridge::CvImage(img_msg->header, "bgr8", vis).toImageMsg());
  }
}

// =====================================================================
// 服务回调
// =====================================================================

void HandEyeCalibrateNode::OnCapture(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (!last_detection_.has_value())
  {
    res->success = false;
    res->message = "未检测到标定板。";
    return;
  }
  const auto& det = last_detection_.value();

  if ((now() - det.stamp).seconds() > max_age_sec_)
  {
    res->success = false;
    res->message = "检测数据过旧。";
    return;
  }
  if (!current_vis_.is_static)
  {
    res->success = false;
    res->message = "拒绝：云台正在运动！";
    return;
  }
  if (det.blur_score < min_blur_score_)
  {
    res->success = false;
    res->message = "拒绝：图像过于模糊！";
    return;
  }

  for (const auto& s : samples_)
  {
    double dist = s.q_gimbal2world.angularDistance(det.q_gimbal2world);
    if (dist < min_angle_dist_rad_)
    {
      std::ostringstream ss;
      ss << "拒绝：与已有样本位姿过近 (" << std::fixed << std::setprecision(1)
         << dist * 57.3 << " deg)。";
      res->success = false;
      res->message = ss.str();
      return;
    }
  }

  samples_.push_back(det);
  std::ostringstream ss;
  ss << "已采集第 " << samples_.size() << " 帧 (RMSE: " << std::fixed
     << std::setprecision(3) << det.reproj_rmse << ")";
  res->success = true;
  res->message = ss.str();
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void HandEyeCalibrateNode::OnReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);
  samples_.clear();
  res->success = true;
  res->message = "已重置所有样本。";
  RCLCPP_INFO(get_logger(), "样本已重置。");
}

void HandEyeCalibrateNode::OnSolve(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (samples_.size() < 5)
  {
    res->success = false;
    res->message =
        "样本不足，至少需要 5 帧（当前: " + std::to_string(samples_.size()) + "）。";
    return;
  }

  if (use_robot_world_)
  {
    SolveRobotWorld(res);
  }
  else
  {
    SolveClassic(res);
  }
}

// =====================================================================
// 求解实现
// =====================================================================

void HandEyeCalibrateNode::SolveClassic(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::vector<cv::Mat> r_g2b, t_g2b, r_t2c, t_t2c;
  for (const auto& s : samples_)
  {
    r_g2b.push_back(s.R_gimbal2world);
    t_g2b.push_back(s.t_gimbal);
    r_t2c.push_back(s.R_target2cam);
    t_t2c.push_back(s.t_target2cam);
  }

  cv::Mat r_cam2grip, t_cam2grip;
  try
  {
    cv::calibrateHandEye(
        r_g2b, t_g2b, r_t2c, t_t2c, r_cam2grip, t_cam2grip,
        static_cast<cv::HandEyeCalibrationMethod>(ToOpenCvMethod(method_)));
  }
  catch (cv::Exception& e)
  {
    res->success = false;
    res->message = std::string("calibrateHandEye 异常: ") + e.what();
    return;
  }

  FormatResult(r_cam2grip, t_cam2grip, "calibrateHandEye (AX=XB)", res);
}

void HandEyeCalibrateNode::SolveRobotWorld(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::vector<cv::Mat> rvecs, tvecs, r_w2g, t_w2g;
  for (const auto& s : samples_)
  {
    cv::Mat rvec;
    cv::Rodrigues(s.R_target2cam, rvec);
    rvecs.push_back(rvec);
    tvecs.push_back(s.t_target2cam);
    r_w2g.push_back(s.R_world2gimbal);
    t_w2g.push_back(s.t_gimbal);
  }

  cv::Mat r_world2board, t_world2board, r_gimbal2camera, t_gimbal2camera;
  try
  {
    cv::calibrateRobotWorldHandEye(rvecs, tvecs, r_w2g, t_w2g, r_world2board,
                                   t_world2board, r_gimbal2camera, t_gimbal2camera);
  }
  catch (cv::Exception& e)
  {
    res->success = false;
    res->message = std::string("calibrateRobotWorldHandEye 异常: ") + e.what();
    return;
  }

  cv::Mat r_camera2gimbal, t_camera2gimbal;
  cv::transpose(r_gimbal2camera, r_camera2gimbal);
  t_camera2gimbal = -r_camera2gimbal * t_gimbal2camera;

  // 标定板位姿信息
  cv::Mat r_board2world, t_board2world;
  cv::transpose(r_world2board, r_board2world);
  t_board2world = -r_board2world * t_world2board;

  Eigen::Matrix3d r_b2w_eigen;
  cv::cv2eigen(r_board2world, r_b2w_eigen);
  Eigen::Vector3d board_ypr = MatToEulers(r_b2w_eigen, 2, 1, 0) * 57.3;

  double bx = t_board2world.at<double>(0);
  double by = t_board2world.at<double>(1);
  double board_dist = std::sqrt(bx * bx + by * by);

  std::ostringstream extra;
  extra << "\n--- 标定板位姿（RobotWorld 联合估计）---\n"
        << "  水平距离: " << std::fixed << std::setprecision(4) << board_dist << " m\n"
        << "  偏角 yaw: " << std::setprecision(2) << board_ypr[0]
        << " deg  pitch: " << board_ypr[1] << " deg  roll: " << board_ypr[2] << " deg\n";

  FormatResult(r_camera2gimbal, t_camera2gimbal, "calibrateRobotWorldHandEye (AX=ZB)",
               res, extra.str());
}

void HandEyeCalibrateNode::FormatResult(
    const cv::Mat& R_cam2gimbal, const cv::Mat& t_cam2gimbal,
    const std::string& method_name,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res,
    const std::string& extra_info)
{
  tf2::Matrix3x3 r_opt;
  r_opt.setRPY(-CV_PI / 2.0, 0.0, -CV_PI / 2.0);

  tf2::Matrix3x3 r_calib(R_cam2gimbal.at<double>(0, 0), R_cam2gimbal.at<double>(0, 1),
                         R_cam2gimbal.at<double>(0, 2), R_cam2gimbal.at<double>(1, 0),
                         R_cam2gimbal.at<double>(1, 1), R_cam2gimbal.at<double>(1, 2),
                         R_cam2gimbal.at<double>(2, 0), R_cam2gimbal.at<double>(2, 1),
                         R_cam2gimbal.at<double>(2, 2));

  tf2::Matrix3x3 r_joint = r_calib * r_opt.transpose();
  double roll = NAN, pitch = NAN, yaw = NAN;
  r_joint.getRPY(roll, pitch, yaw);

  double tx = t_cam2gimbal.at<double>(0);
  double ty = t_cam2gimbal.at<double>(1);
  double tz = t_cam2gimbal.at<double>(2);

  Eigen::Matrix3d r_cam2gimbal_eigen;
  cv::cv2eigen(R_cam2gimbal, r_cam2gimbal_eigen);
  Eigen::Matrix3d r_gimbal2ideal;
  r_gimbal2ideal << 0, -1, 0, 0, 0, -1, 1, 0, 0;
  Eigen::Matrix3d r_cam2ideal = r_gimbal2ideal * r_cam2gimbal_eigen;
  Eigen::Vector3d cam_ypr = MatToEulers(r_cam2ideal, 1, 0, 2) * 57.3;

  std::ostringstream ss;
  ss << std::fixed;

  ss << "╔══════════════════════════════════════════════════╗\n"
     << "║  " << method_name << "\n"
     << "║  样本数: " << samples_.size() << "\n"
     << "╠══════════════════════════════════════════════════╣\n"
     << "║\n"
     << "║    <xacro:arg name=\"xyz\" default=\"" << std::setprecision(6) << tx << " "
     << ty << " " << tz << "\" />\n"
     << "║    <xacro:arg name=\"rpy\" default=\"" << std::setprecision(6) << roll << " "
     << pitch << " " << yaw << "\" />\n"
     << "║\n"
     << "║  或 launch 传参:\n"
     << "║    xyz:=" << std::setprecision(6) << tx << " " << ty << " " << tz << "\n"
     << "║    rpy:=" << std::setprecision(6) << roll << " " << pitch << " " << yaw << "\n"
     << "║\n"
     << "╠══════════════════════════════════════════════════╣\n"
     << "║  相机安装偏角（相对于正装的偏差，仅供参考）\n"
     << "║    yaw: " << std::setprecision(2) << cam_ypr[0]
     << " deg  pitch: " << cam_ypr[1] << " deg  roll: " << cam_ypr[2] << " deg\n"
     << "╚══════════════════════════════════════════════════╝\n";

  if (!extra_info.empty()) ss << extra_info;

  res->success = true;
  res->message = ss.str();
  RCLCPP_INFO(get_logger(), "\n%s", ss.str().c_str());
}
// =====================================================================
// 注册组件
// =====================================================================
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(HandEyeCalibrateNode)
