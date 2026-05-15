
#include "armor_detector/detector_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.hpp>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
  auto detector_type =
      this->declare_parameter("detector_type", std::string("traditional"));
  detector_ = create_detector(detector_type, *this);

  // Pose optimizer
  pose_optimizer_ = InitPoseOptimizer();

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
      "/detector/armors", rclcpp::SensorDataQoS());

  // Debug Publishers
  debug_ = this->declare_parameter("debug", false);
  if (debug_)
  {
    CreateDebugPublishers();
  }

  // Debug param change monitor
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
      "debug",
      [this](const rclcpp::Parameter& p)
      {
        debug_ = p.as_bool();
        debug_ ? CreateDebugPublishers() : DestroyDebugPublishers();
      });

  // 创建相机内参订阅者 → 接收一次/camera_info消息 → 提取图像中心、保存内参、初始化 PnP
  // 求解器 → 停止订阅
  auto robot_type = this->declare_parameter<std::string>("robot_type", "default");

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this, robot_type](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info)
      {
        if (current_frame_id_ != "" && camera_info->header.frame_id == current_frame_id_)
        {
          return;
        }
        cam_center_ = cv::Point2f(static_cast<float>(camera_info->k[2]),
                                  static_cast<float>(camera_info->k[5]));
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        if (pose_optimizer_)
        {
          pose_optimizer_->SetCameraIntrinsics(
              cv::Mat(3, 3, CV_64F, const_cast<double*>(cam_info_->k.data())),
              cv::Mat(cam_info_->d));
        }
        if (!pnp_solver_)
        {
          pnp_solver_ = InitPnPSolver();
          if (robot_type != "hero")
          {
            cam_info_sub_.reset();  // 只需要接收第一条相机内参消息，之后就可以停掉订阅了
          }
        }
        else
        {
          pnp_solver_->SetCameraInfo(cam_info_->k, cam_info_->d);
        }
        current_frame_id_ = camera_info->header.frame_id;
        RCLCPP_INFO(this->get_logger(), "PnP solver updated (frame_id: %s)",
                    camera_info->header.frame_id.c_str());
      });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::ImageCallback, this, std::placeholders::_1));
}

namespace
{
double compute_view_yaw_deg(const cv::Mat& rvec, const cv::Mat& tvec)
{
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  // 装甲板法向
  cv::Vec3d normal_cam(rmat.at<double>(0, 0), rmat.at<double>(1, 0),
                       rmat.at<double>(2, 0));

  cv::Vec3d center_dir(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                       tvec.at<double>(2, 0));

  double n_norm = cv::norm(normal_cam);
  double d_norm = cv::norm(center_dir);
  if (n_norm < 1e-6 || d_norm < 1e-6)
  {
    return 180.0;
  }

  double cos_theta = (normal_cam[0] * center_dir[0] + normal_cam[1] * center_dir[1] +
                      normal_cam[2] * center_dir[2]) /
                     (n_norm * d_norm);

  cos_theta = std::clamp(std::abs(cos_theta), 0.0, 1.0);

  return std::acos(cos_theta) * 180.0 / M_PI;
}
}  // namespace

void ArmorDetectorNode::ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  debug_latencies_.clear();  // 初始化时清空 debug_latencies_
  auto start_time = this->get_clock()->now();
  if (!pnp_solver_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "Waiting for /camera_info, skip current image.");
    return;
  }

  auto armors = DetectArmors(img_msg);

  if (pnp_solver_ != nullptr)
  {
    auto pnp_start_time = this->get_clock()->now();
    armors_msg_.header = img_msg->header;
    armors_msg_.armors.clear();

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto& armor : armors)
    {
      cv::Mat rvec, tvec;
      bool success = pnp_solver_->SolvePnP(armor, rvec, tvec);
      if (success)
      {
        if (pose_optimizer_)
        {
          try
          {
            auto odom_to_camera_optical = tf2_buffer_->lookupTransform(
                "gimbal_odom", img_msg->header.frame_id, img_msg->header.stamp,
                rclcpp::Duration::from_seconds(0.01));

            Eigen::Quaterniond q;
            tf2::fromMsg(odom_to_camera_optical.transform.rotation, q);
            Eigen::Matrix3d rmat_gimbal_cam = q.toRotationMatrix();

            auto odom_to_gimbal = tf2_buffer_->lookupTransform(
                "gimbal_odom", "pitch_link", img_msg->header.stamp,
                rclcpp::Duration::from_seconds(0.01));

            tf2::Quaternion qu(
                odom_to_gimbal.transform.rotation.x, odom_to_gimbal.transform.rotation.y,
                odom_to_gimbal.transform.rotation.z, odom_to_gimbal.transform.rotation.w);
            tf2::Matrix3x3 m(qu);
            double roll = NAN, pitch = NAN, yaw = NAN;
            m.getRPY(roll, pitch, yaw);

            pose_optimizer_->SetCameraToGimbalRotation(rmat_gimbal_cam);
            pose_optimizer_->SetArmorNumber(armor.number == "outpost");

            // TF 查询成功后才执行优化；失败时 rvec/tvec 保持原始 PnP 结果不变
            if (!pose_optimizer_->Optimize(armor, rvec, tvec))
            {
              RCLCPP_DEBUG(this->get_logger(),
                           "Pose optimization failed, using original PnP result.");
            }
          }
          catch (const tf2::TransformException& ex)
          {
            // RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
          }
        }

        // cv::Mat R;
        // cv::Rodrigues(rvec, R);
        // double yaw = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2));
        // double yaw_deg = yaw * 180.0 / CV_PI;
        // RCLCPP_WARN(this->get_logger(), "Armor view: %.2f rad, %.2f deg", yaw,
        // yaw_deg);

        // double view_yaw_deg = compute_view_yaw_deg(rvec, tvec);
        // if (view_yaw_deg > 55.0)
        // {
        //   RCLCPP_WARN(this->get_logger(), "Drop armor by yaw gate: %.2f deg > %.2f deg",
        //               view_yaw_deg, 55.0);
        //   continue;
        // }
        // else
        // {
        //   RCLCPP_WARN(this->get_logger(), "View yaw: %.2f deg", view_yaw_deg);
        // }

        // Fill basic info
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
            rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
            rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
            rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // Fill the distance to image center
        armor_msg.distance_to_image_center =
            pnp_solver_->CalculateDistanceToCenter(armor.center);

        armors_msg_.armors.emplace_back(armor_msg);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }
    auto pnp_end_time = this->get_clock()->now();
    auto pnp_latency =
        static_cast<uint64_t>((pnp_end_time - pnp_start_time).nanoseconds() / 1000);
    // debug_latencies_.emplace_back("PnP", static_cast<uint64_t>(pnp_latency));
    // RCLCPP_INFO(this->get_logger(), "PnP latency: %lu us", pnp_latency);

    // Publishing detected armors
    armors_pub_->publish(armors_msg_);
  }
}

std::vector<Armor> ArmorDetectorNode::DetectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  auto convert_start_time = this->get_clock()->now();
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
  auto convert_end_time = this->get_clock()->now();
  auto convert_latency =
      static_cast<uint64_t>((convert_end_time - convert_start_time).nanoseconds() / 1000);
  debug_latencies_.emplace_back("Image Convert", static_cast<uint64_t>(convert_latency));
  DetectionResult result = detector_->Detect(img);
  auto detect_latencies = detector_->GetDebugLatencies();
  debug_latencies_.insert(debug_latencies_.end(), detect_latencies.begin(),
                          detect_latencies.end());

  auto final_time = this->now();

  if (debug_)
  {
    auto debug_data_fill_start_time = std::chrono::steady_clock::now();
    if (result.binary_image)
    {
      binary_img_pub_.publish(
          cv_bridge::CvImage(img_msg->header, "mono8", *result.binary_image)
              .toImageMsg());
    }

    if (result.debug_data)
    {
      lights_data_pub_->publish(result.debug_data->lights);
      armors_data_pub_->publish(result.debug_data->armors);
    }

    if (result.numbers_image && !result.numbers_image->empty())
    {
      number_img_pub_.publish(
          *cv_bridge::CvImage(img_msg->header, "mono8", *result.numbers_image)
               .toImageMsg());
    }

    detector_->DrawResults(img);

    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    // std::stringstream latency_ss;
    // latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    // auto latency_s = latency_ss.str();
    // cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
    //             cv::Scalar(0, 255, 0), 2);
    auto debug_data_fill_end_time = std::chrono::steady_clock::now();
    auto debug_data_fill_latency =
        std::chrono::duration_cast<std::chrono::microseconds>(debug_data_fill_end_time -
                                                              debug_data_fill_start_time)
            .count();
    debug_latencies_.emplace_back("Debug Fill",
                                  static_cast<uint64_t>(debug_data_fill_latency));

    // 在图像上显示各环节延迟
    int y_offset = 30;
    for (const auto& [stage, stage_latency] : debug_latencies_)
    {
      std::string text;
      text += stage + ": " + std::to_string(stage_latency) + "us";
      cv::putText(img, text, cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 255, 0), 1);
      y_offset += 20;
    }
    result_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }
  // RCLCPP_INFO(this->get_logger(), "latencies size: %zu", debug_latencies_.size());
  return std::move(result.armors);
}

std::unique_ptr<PnPSolver> ArmorDetectorNode::InitPnPSolver()
{
  PnPSolver::PnpFilterParams pnp_filter_params{
      .new_pnp_filter_method =
          this->declare_parameter("pnp_filter.use_new_pnp_filter_method", false),
      .max_normal_dot = this->declare_parameter("pnp_filter.max_normal_dot", 0.0),
      .reproj_weight = this->declare_parameter("pnp_filter.reproj_weight", 1.0),
      .normal_weight = this->declare_parameter("pnp_filter.normal_weight", 0.5)};
  return std::make_unique<PnPSolver>(cam_info_->k, cam_info_->d, pnp_filter_params);
}

std::unique_ptr<ArmorPoseOptimizer> ArmorDetectorNode::InitPoseOptimizer()
{
  if (!this->declare_parameter("optimizer.use_pose_optimizer", false))
  {
    return nullptr;
  }

  // Determine optimization method from parameter
  std::string optimize_method_str =
      this->declare_parameter("optimizer.optimize_method", std::string("RANGE_SHORT_LM"));
  ArmorPoseOptimizer::Params::OptimizeMethod optimize_method =
      ArmorPoseOptimizer::Params::OptimizeMethod::RANGE_LM;  // default
  if (optimize_method_str == "LM")
  {
    optimize_method = ArmorPoseOptimizer::Params::OptimizeMethod::LM;
  }
  else if (optimize_method_str == "RANGE")
  {
    optimize_method = ArmorPoseOptimizer::Params::OptimizeMethod::RANGE;
  }
  else if (optimize_method_str == "RANGE_SHORT_LM")
  {
    optimize_method = ArmorPoseOptimizer::Params::OptimizeMethod::RANGE_LM;
  }

  // Initialize optimizer parameters using designated initializers
  ArmorPoseOptimizer::Params opt_params{
      .optimize_method = optimize_method,
      .standard_pitch_deg = this->declare_parameter("optimizer.standard_pitch_deg", 15.0),
      .outpost_pitch_deg = this->declare_parameter("optimizer.outpost_pitch_deg", -15.0),
      .max_pitch_deviation =
          this->declare_parameter("optimizer.max_pitch_deviation", 15.0),
      .max_roll_deviation = this->declare_parameter("optimizer.max_roll_deviation", 15.0),
      .max_iterations =
          static_cast<int>(this->declare_parameter("optimizer.max_iterations", 20)),
      .range_fix_t_cam_ = this->declare_parameter("optimizer.range_fix_t_cam", false),
      .range_search_half_range_deg =
          this->declare_parameter("optimizer.range_search_half_range_deg", 70.0),
      .range_search_coarse_step_deg =
          this->declare_parameter("optimizer.range_search_coarse_step_deg", 1.0),
      .range_search_fine_range_deg =
          this->declare_parameter("optimizer.range_search_fine_range_deg", 2.0),
      .range_search_fine_step_deg =
          this->declare_parameter("optimizer.range_search_fine_step_deg", 0.1)};

  InitTransformListener();

  return std::make_unique<ArmorPoseOptimizer>(opt_params);
}

void ArmorDetectorNode::InitTransformListener()
{
  odom_frame_ = this->declare_parameter("odom_frame", "gimbal_odom");
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

void ArmorDetectorNode::CreateDebugPublishers()
{
  lights_data_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugLights>(
      "/detector/debug_lights", 10);
  armors_data_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugArmors>(
      "/detector/debug_armors", 10);

  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

void ArmorDetectorNode::DestroyDebugPublishers()
{
  lights_data_pub_.reset();  // 释放调试发布者所占用的资源，并将其置为无效状态
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();  // 关闭调试发布者，使其不再发布任何消息
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its
// library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)