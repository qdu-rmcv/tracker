#include "armor_tracker/tracker_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions& options)
    : Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // 通用配置
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);
  const auto robot_type = this->declare_parameter<std::string>("robot_type", "default");
  is_hero_ = (robot_type == "hero");

  // 读取所有 Tracker 参数
  TrackerParams params = DeclareTrackerParams();
  lost_time_thres_ = params.lost_time_thres;
  change_time_thres_ = params.change_time_thres;

  // 构造 Tracker（内部会构造三个 EKF）
  tracker_ = std::make_unique<Tracker>(params);

  // ---------- TF / 订阅 ----------
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  armors_filter_ = std::make_shared<armors_tf2_filter>(
      armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
      this->get_node_clock_interface(), std::chrono::duration<int>(1));

  armors_filter_->registerCallback(&ArmorTrackerNode::ArmorsCallback, this);

  // ---------- 发布 ----------
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>(
      "/tracker/info", 10);
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
      "/tracker/target", rclcpp::SensorDataQoS());
}

// =====================================================================
// 参数读取
// =====================================================================
TrackerParams ArmorTrackerNode::DeclareTrackerParams()
{
  TrackerParams p;

  // 匹配
  p.max_match_distance =
      this->declare_parameter("tracker.max_match_distance", p.max_match_distance);
  p.max_match_yaw_diff =
      this->declare_parameter("tracker.max_match_yaw_diff", p.max_match_yaw_diff);
  p.tracking_thres =
      static_cast<int>(this->declare_parameter("tracker.tracking_thres", p.tracking_thres));
  p.lost_time_thres =
      this->declare_parameter("tracker.lost_time_thres", p.lost_time_thres);
  p.change_time_thres =
      this->declare_parameter("tracker.change_time_thres", p.change_time_thres);

  // 模型选择迟滞
  p.v_yaw_armor_threshold =
      this->declare_parameter("tracker.v_yaw_armor_threshold", p.v_yaw_armor_threshold);
  p.v_yaw_full_threshold =
      this->declare_parameter("tracker.v_yaw_full_threshold", p.v_yaw_full_threshold);

  // 半径
  p.radius_min = this->declare_parameter("tracker.radius_min", p.radius_min);
  p.radius_max = this->declare_parameter("tracker.radius_max", p.radius_max);
  p.default_init_radius =
      this->declare_parameter("tracker.default_init_radius", p.default_init_radius);

  // 整车 EKF Q
  p.s2_q_x_full = this->declare_parameter("ekf.s2_q_x_full", p.s2_q_x_full);
  p.s2_q_y_full = this->declare_parameter("ekf.s2_q_y_full", p.s2_q_y_full);
  p.s2_q_z_full = this->declare_parameter("ekf.s2_q_z_full", p.s2_q_z_full);
  p.s2_q_yaw_full = this->declare_parameter("ekf.s2_q_yaw_full", p.s2_q_yaw_full);
  p.s2_q_r_full = this->declare_parameter("ekf.s2_q_r_full", p.s2_q_r_full);

  // 装甲板 CV EKF Q
  p.s2_q_x_armor = this->declare_parameter("ekf.s2_q_x_armor", p.s2_q_x_armor);
  p.s2_q_y_armor = this->declare_parameter("ekf.s2_q_y_armor", p.s2_q_y_armor);
  p.s2_q_z_armor = this->declare_parameter("ekf.s2_q_z_armor", p.s2_q_z_armor); 

  // 前哨 EKF Q
  p.s2_q_xy_outpost =
      this->declare_parameter("ekf.s2_q_xy_outpost", p.s2_q_xy_outpost);
  p.s2_q_z_outpost =
      this->declare_parameter("ekf.s2_q_z_outpost", p.s2_q_z_outpost);
  p.s2_q_yaw_outpost =
      this->declare_parameter("ekf.s2_q_yaw_outpost", p.s2_q_yaw_outpost);

  // 前哨常量 / 行为
  p.outpost_r = this->declare_parameter("outpost.outpost_r", p.outpost_r);
  p.outpost_dz = this->declare_parameter("outpost.outpost_dz", p.outpost_dz);
  p.outpost_cast_threshold =
      this->declare_parameter("outpost.outpost_cast_threshold", p.outpost_cast_threshold);
  p.outpost_vyaw_abs = this->declare_parameter("outpost.outpost_vyaw_abs", p.outpost_vyaw_abs);
  p.outpost_static_threshold =
      this->declare_parameter("outpost.outpost_static_threshold", p.outpost_static_threshold);
  p.outpost_learning_frames = static_cast<int>(
      this->declare_parameter("outpost.outpost_learning_frames", p.outpost_learning_frames));
  p.outpost_zc_stable_count = static_cast<int>(
      this->declare_parameter("outpost.outpost_zc_stable_count", p.outpost_zc_stable_count));
  p.outpost_idx_geo_margin =
      this->declare_parameter("outpost.outpost_idx_geo_margin", p.outpost_idx_geo_margin);

  // 测量噪声
  p.r_ypd_yaw_std = this->declare_parameter("ekf.r_ypd_yaw_std", p.r_ypd_yaw_std);
  p.r_ypd_pitch_std =
      this->declare_parameter("ekf.r_ypd_pitch_std", p.r_ypd_pitch_std);
  p.r_ypd_distance_std_scale = this->declare_parameter("ekf.r_ypd_distance_std_scale",
                                                       p.r_ypd_distance_std_scale);
  p.r_armor_yaw_std =
      this->declare_parameter("ekf.r_armor_yaw_std", p.r_armor_yaw_std);

  return p;
}

// =====================================================================
// Armors 回调
// =====================================================================
void ArmorTrackerNode::ArmorsCallback(
    auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // 1. 查询相机位姿（target_frame -> camera frame）
  Eigen::Matrix3d camera_to_world_rot = Eigen::Matrix3d::Identity();
  Eigen::Vector3d camera_origin_world = Eigen::Vector3d::Zero();
  try
  {
    const auto tf = tf2_buffer_->lookupTransform(
        target_frame_, armors_msg->header.frame_id, armors_msg->header.stamp);

    const auto& t = tf.transform.translation;
    const auto& q = tf.transform.rotation;

    camera_origin_world = Eigen::Vector3d(t.x, t.y, t.z);
    camera_to_world_rot =
        Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to query camera pose: %s", ex.what());
    return;
  }

  tracker_->SetCameraPose(camera_to_world_rot, camera_origin_world);

  if (is_hero_ && armors_msg->header.frame_id != last_camera_frame_id_)
  {
    last_camera_frame_id_ = armors_msg->header.frame_id;
  }

  // 2. 装甲板位姿统一转到 target_frame
  for (auto& armor : armors_msg->armors)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try
    {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    }
    catch (const tf2::ExtrapolationException& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // 3. 过滤异常
  armors_msg->armors.erase(
      std::remove_if(
          armors_msg->armors.begin(), armors_msg->armors.end(),
          [this](const auto_aim_interfaces::msg::Armor& armor) {
            return std::fabs(armor.pose.position.z) > 5 ||
                   Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                       max_armor_distance_;
          }),
      armors_msg->armors.end());

  // 4. 状态机调度
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  if (tracker_->GetTrackerState() == Tracker::State::LOST)
  {
    RCLCPP_ERROR(get_logger(), "Tracker state is LOST");
    tracker_->Init(armors_msg);
    target_msg.tracking = false;
  }
  else
  {
    dt_ = (time - last_time_).seconds();
    dt_ = std::clamp(dt_, 1e-3, 0.1);

    tracker_->SetDt(dt_);
    tracker_->SetTimingThres(static_cast<int>(lost_time_thres_ / dt_),
                             static_cast<int>(change_time_thres_ / dt_));
    tracker_->Update(armors_msg);

    const auto state = tracker_->GetTrackerState();
    if (state == Tracker::State::DETECTING)
    {
      target_msg.tracking = false;
      RCLCPP_ERROR(get_logger(),
                   "Tracker state is DETECTING");
    }
    else if (state == Tracker::State::TRACKING || state == Tracker::State::TEMP_LOST)
    {
      target_msg.tracking = true;

      const auto out = tracker_->GetOutput();
      target_msg.type = out.armor_type;
      target_msg.armors_num = out.armors_num;
      target_msg.position.x = out.position.x();
      target_msg.position.y = out.position.y();
      target_msg.position.z = out.position.z();
      target_msg.velocity.x = out.velocity.x();
      target_msg.velocity.y = out.velocity.y();
      target_msg.velocity.z = out.velocity.z();
      target_msg.yaw = out.yaw;
      target_msg.v_yaw = out.v_yaw;
      target_msg.radius_1 = out.radius_1;
      target_msg.radius_2 = out.radius_2;
      target_msg.dz = out.dz;
      target_msg.outpost_idx = out.outpost_idx;
      target_msg.is_center = out.is_center;

      signed char num = 0;
      if (out.armor_number == "outpost")
      {
        num = 10;
      }
      else if (out.armor_number == "guard")
      {
        num = 7;
      }
      else if (out.armor_number == "base")
      {
        num = 11;
      }
      else if (!out.armor_number.empty())
      {
        num = static_cast<signed char>(std::stoi(out.armor_number));
      }
      target_msg.num = num;
    }
  }
  target_msg.state = static_cast<int8_t>(tracker_->GetTrackerState());

  last_time_ = time;
  target_pub_->publish(target_msg);

  // 5. 调试 info
  info_msg.position_diff = tracker_->info_position_diff;
  info_msg.yaw_diff = tracker_->info_yaw_diff;
  info_msg.position.x = tracker_->measurement(0);
  info_msg.position.y = tracker_->measurement(1);
  info_msg.position.z = tracker_->measurement(2);
  info_msg.yaw = tracker_->measurement(3);
  info_msg.outpost_idx = tracker_->GetOutput().outpost_idx;
  info_pub_->publish(info_msg);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
