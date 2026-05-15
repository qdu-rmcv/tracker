#include "armor_tracker/tracker.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <numbers>
#include <rclcpp/logger.hpp>

namespace rm_auto_aim
{

// =====================================================================
// 构造
// =====================================================================
Tracker::Tracker(const TrackerParams& params)
    : params_(params), full_pred_(Eigen::VectorXd::Zero(8))
{
  another_r_ = params_.default_init_radius;
  full_last_r_ = params_.default_init_radius;

  BuildEkfFull();
  BuildEkfArmor();
  BuildEkfOutpost();
}

void Tracker::SetCameraPose(const Eigen::Matrix3d& rot_camera_to_world,
                            const Eigen::Vector3d& camera_origin_world)
{
  camera_to_world_rot_ = rot_camera_to_world;
  camera_origin_world_ = camera_origin_world;
}

// =====================================================================
// EKF 构建：整车 (8 维)
//   state: [xc, v_xc, yc, v_yc, za, yaw, v_yaw, r]
//   meas : [xa, ya, za, yaw]   (4 维)
// =====================================================================
void Tracker::BuildEkfFull()
{
  // f
  auto f = [this](const Eigen::VectorXd& x)
  {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;  // xc
    x_new(2) += x(3) * dt_;  // yc
    x_new(5) += x(6) * dt_;  // yaw
    return x_new;
  };

  // J_f (8x8)
  auto j_f = [this](const Eigen::VectorXd&)
  {
    Eigen::MatrixXd m = Eigen::MatrixXd::Identity(8, 8);
    m(0, 1) = dt_;
    m(2, 3) = dt_;
    m(5, 6) = dt_;
    return m;
  };

  // h: 装甲板 = 中心 - r*(cos(yaw),sin(yaw)); za=za; yaw_obs=yaw
  auto h = [](const Eigen::VectorXd& x)
  {
    Eigen::VectorXd z(4);
    const double xc = x(0), yc = x(2), za = x(4), yaw = x(5), r = x(7);
    z(0) = xc - r * std::cos(yaw);
    z(1) = yc - r * std::sin(yaw);
    z(2) = za;
    z(3) = yaw;
    return z;
  };

  // J_h (4x8)
  auto j_h = [](const Eigen::VectorXd& x)
  {
    const double yaw = x(5), r = x(7);
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(4, 8);
    // 列序: xc, vxc, yc, vyc, za, yaw, vyaw, r
    m(0, 0) = 1;
    m(0, 5) = r * std::sin(yaw);
    m(0, 7) = -std::cos(yaw);
    m(1, 2) = 1;
    m(1, 5) = -r * std::cos(yaw);
    m(1, 7) = -std::sin(yaw);
    m(2, 4) = 1;
    m(3, 5) = 1;
    return m;
  };

  // Q (8x8)：xy/yaw 用 CV block；za/r 用随机游走
  auto u_q = [this]()
  {
    const double t = dt_;
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(8, 8);
    auto add_cv = [&](int p, int v, double s2)
    {
      const double a = std::pow(t, 4) / 4.0 * s2;
      const double b = std::pow(t, 3) / 2.0 * s2;
      const double c = std::pow(t, 2) * s2;
      q(p, p) = a;
      q(p, v) = b;
      q(v, p) = b;
      q(v, v) = c;
    };
    add_cv(0, 1, params_.s2_q_x_full);
    add_cv(2, 3, params_.s2_q_y_full);
    add_cv(5, 6, params_.s2_q_yaw_full);
    q(4, 4) = t * params_.s2_q_z_full;
    q(7, 7) = t * params_.s2_q_r_full;
    return q;
  };

  // R (4x4)：ypd 在世界系下转化为 xyz 协方差，再加 yaw_armor 噪声
  auto u_r = [this](const Eigen::VectorXd& x)
  {
    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(4, 4);

    const double xc = x(0), yc = x(2), za = x(4), yaw = x(5), radius = x(7);
    const Eigen::Vector3d p_world(xc - radius * std::cos(yaw),
                                  yc - radius * std::sin(yaw), za);

    const Eigen::Vector3d p_cam =
        camera_to_world_rot_.transpose() * (p_world - camera_origin_world_);
    const double dist = std::max(1e-6, p_cam.norm());

    const double s_yaw = params_.r_ypd_yaw_std;
    const double s_pitch = params_.r_ypd_pitch_std;
    const double s_dist = params_.r_ypd_distance_std_scale * dist * dist;

    Eigen::Matrix3d cov_ypd = Eigen::Matrix3d::Zero();
    cov_ypd(0, 0) = s_yaw * s_yaw;
    cov_ypd(1, 1) = s_pitch * s_pitch;
    cov_ypd(2, 2) = s_dist * s_dist;

    const Eigen::Matrix3d j = BuildJacobianYpdToCameraXyz(p_cam);
    const Eigen::Matrix3d cov_cam = j * cov_ypd * j.transpose();
    const Eigen::Matrix3d cov_world =
        camera_to_world_rot_ * cov_cam * camera_to_world_rot_.transpose();

    r.block<3, 3>(0, 0) = cov_world;
    r(3, 3) = 0.005 * std::log1p(dist) + 0.09;
    return r;
  };

  Eigen::MatrixXd p0 = Eigen::MatrixXd::Zero(8, 8);
  p0(0, 0) = 0.05;
  p0(1, 1) = 1.0;
  p0(2, 2) = 0.05;
  p0(3, 3) = 1.0;
  p0(4, 4) = 0.05;
  p0(5, 5) = 0.1;
  p0(6, 6) = 2.0;
  p0(7, 7) = 1.0;

  ekf_full_ = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
}

// =====================================================================
// EKF 构建：装甲板 CV (5 维)
//   state: [xa, v_xa, ya, v_ya, za]
//   meas : [xa, ya, za]   (3 维)
// =====================================================================
void Tracker::BuildEkfArmor()
{
  auto f = [this](const Eigen::VectorXd& x)
  {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    return x_new;
  };

  auto j_f = [this](const Eigen::VectorXd&)
  {
    Eigen::MatrixXd m = Eigen::MatrixXd::Identity(5, 5);
    m(0, 1) = dt_;
    m(2, 3) = dt_;
    return m;
  };

  auto h = [](const Eigen::VectorXd& x)
  {
    Eigen::VectorXd z(3);
    z(0) = x(0);
    z(1) = x(2);
    z(2) = x(4);
    return z;
  };

  auto j_h = [](const Eigen::VectorXd&)
  {
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(3, 5);
    m(0, 0) = 1;
    m(1, 2) = 1;
    m(2, 4) = 1;
    return m;
  };

  auto u_q = [this]()
  {
    const double t = dt_;
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(5, 5);
    auto add_cv = [&](int p, int v, double s2)
    {
      const double a = std::pow(t, 4) / 4.0 * s2;
      const double b = std::pow(t, 3) / 2.0 * s2;
      const double c = std::pow(t, 2) * s2;
      q(p, p) = a;
      q(p, v) = b;
      q(v, p) = b;
      q(v, v) = c;
    };
    add_cv(0, 1, params_.s2_q_x_armor);
    add_cv(2, 3, params_.s2_q_y_armor);
    q(4, 4) = t * params_.s2_q_z_armor;
    return q;
  };

  // R (3x3)：复用 ypd 协方差，去掉 yaw 行列
  auto u_r = [this](const Eigen::VectorXd& x)
  {
    const Eigen::Vector3d p_world(x(0), x(2), x(4));
    const Eigen::Vector3d p_cam =
        camera_to_world_rot_.transpose() * (p_world - camera_origin_world_);
    const double dist = std::max(1e-6, p_cam.norm());

    const double s_yaw = params_.r_ypd_yaw_std;
    const double s_pitch = params_.r_ypd_pitch_std;
    const double s_dist = params_.r_ypd_distance_std_scale * dist * dist;

    Eigen::Matrix3d cov_ypd = Eigen::Matrix3d::Zero();
    cov_ypd(0, 0) = s_yaw * s_yaw;
    cov_ypd(1, 1) = s_pitch * s_pitch;
    cov_ypd(2, 2) = s_dist * s_dist;

    const Eigen::Matrix3d j = BuildJacobianYpdToCameraXyz(p_cam);
    const Eigen::Matrix3d cov_cam = j * cov_ypd * j.transpose();
    const Eigen::Matrix3d cov_world =
        camera_to_world_rot_ * cov_cam * camera_to_world_rot_.transpose();

    Eigen::MatrixXd r = cov_world;
    return r;
  };

  Eigen::MatrixXd p0 = Eigen::MatrixXd::Zero(5, 5);
  p0(0, 0) = 0.05;
  p0(1, 1) = 1.0;
  p0(2, 2) = 0.05;
  p0(3, 3) = 1.0;
  p0(4, 4) = 0.05;

  ekf_armor_ = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
}

// =====================================================================
// EKF 构建：前哨站 (5 维)
//   state: [xc, yc, zc, yaw, v_yaw]
//   meas : [xa, ya, za_corrected, yaw]   (送入前已修正 z)
// =====================================================================
void Tracker::BuildEkfOutpost()
{
  auto f = [this](const Eigen::VectorXd& x)
  {
    Eigen::VectorXd x_new = x;
    x_new(3) += x(4) * dt_;  // yaw += v_yaw * dt
    return x_new;
  };

  auto j_f = [this](const Eigen::VectorXd&)
  {
    Eigen::MatrixXd m = Eigen::MatrixXd::Identity(5, 5);
    m(3, 4) = dt_;
    return m;
  };

  const double r_const = params_.outpost_r;

  auto h = [r_const](const Eigen::VectorXd& x)
  {
    Eigen::VectorXd z(4);
    const double xc = x(0), yc = x(1), zc = x(2), yaw = x(3);
    z(0) = xc - r_const * std::cos(yaw);
    z(1) = yc - r_const * std::sin(yaw);
    z(2) = zc;
    z(3) = yaw;
    return z;
  };

  auto j_h = [r_const](const Eigen::VectorXd& x)
  {
    const double yaw = x(3);
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(4, 5);
    m(0, 0) = 1;
    m(0, 3) = r_const * std::sin(yaw);
    m(1, 1) = 1;
    m(1, 3) = -r_const * std::cos(yaw);
    m(2, 2) = 1;
    m(3, 3) = 1;
    return m;
  };

  auto u_q = [this]()
  {
    const double t = dt_;
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(5, 5);
    q(0, 0) = t * params_.s2_q_xy_outpost;
    q(1, 1) = t * params_.s2_q_xy_outpost;
    q(2, 2) = t * params_.s2_q_z_outpost;
    const double a = std::pow(t, 4) / 4.0 * params_.s2_q_yaw_outpost;
    const double b = std::pow(t, 3) / 2.0 * params_.s2_q_yaw_outpost;
    const double c = std::pow(t, 2) * params_.s2_q_yaw_outpost;
    q(3, 3) = a;
    q(3, 4) = b;
    q(4, 3) = b;
    q(4, 4) = c;
    return q;
  };

  auto u_r = [this, r_const](const Eigen::VectorXd& x)
  {
    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(4, 4);
    const double xc = x(0), yc = x(1), zc = x(2), yaw = x(3);
    const Eigen::Vector3d p_world(xc - r_const * std::cos(yaw),
                                  yc - r_const * std::sin(yaw), zc);
    const Eigen::Vector3d p_cam =
        camera_to_world_rot_.transpose() * (p_world - camera_origin_world_);
    const double dist = std::max(1e-6, p_cam.norm());

    const double s_yaw = params_.r_ypd_yaw_std;
    const double s_pitch = params_.r_ypd_pitch_std;
    const double s_dist = params_.r_ypd_distance_std_scale * dist * dist;

    Eigen::Matrix3d cov_ypd = Eigen::Matrix3d::Zero();
    cov_ypd(0, 0) = s_yaw * s_yaw;
    cov_ypd(1, 1) = s_pitch * s_pitch;
    cov_ypd(2, 2) = s_dist * s_dist;

    const Eigen::Matrix3d j = BuildJacobianYpdToCameraXyz(p_cam);
    const Eigen::Matrix3d cov_cam = j * cov_ypd * j.transpose();
    const Eigen::Matrix3d cov_world =
        camera_to_world_rot_ * cov_cam * camera_to_world_rot_.transpose();

    r.block<3, 3>(0, 0) = cov_world;
    r(3, 3) = 0.005 * std::log1p(dist) + 0.09;
    return r;
  };

  Eigen::MatrixXd p0 = Eigen::MatrixXd::Zero(5, 5);
  p0(0, 0) = 0.05;
  p0(1, 1) = 0.05;
  p0(2, 2) = 0.05;
  p0(3, 3) = 0.1;
  p0(4, 4) = 2.0;

  ekf_outpost_ = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
}

// =====================================================================
// 工具：ypd → camera xyz 雅可比
// =====================================================================
Eigen::Matrix3d Tracker::BuildJacobianYpdToCameraXyz(const Eigen::Vector3d& p_cam) const
{
  const double X = p_cam.x();
  const double Y = p_cam.y();
  const double Z = p_cam.z();

  const double D = std::max(1e-6, p_cam.norm());
  const double RHO = std::max(1e-6, std::hypot(X, Z));
  const double YAW = std::atan2(X, Z);
  const double PITCH = std::atan2(Y, RHO);

  const double CY = std::cos(YAW);
  const double SY = std::sin(YAW);
  const double CP = std::cos(PITCH);
  const double SP = std::sin(PITCH);

  Eigen::Matrix3d j;
  j << D * CP * CY, -D * SP * SY, CP * SY, 0.0, D * CP, SP, -D * CP * SY, -D * SP * CY,
      CP * CY;
  return j;
}

// =====================================================================
// 入口：Init / Update
// =====================================================================
void Tracker::Init(const Armors::SharedPtr& armors_msg)
{
  if (armors_msg->armors.empty())
  {
    return;
  }

  double min_distance = DBL_MAX;
  Armor chosen = armors_msg->armors[0];
  for (const auto& a : armors_msg->armors)
  {
    if (a.distance_to_image_center < min_distance)
    {
      min_distance = a.distance_to_image_center;
      chosen = a;
    }
  }

  tracked_armor_ = chosen;
  tracked_id_ = chosen.number;
  is_outpost_ = (chosen.number == "outpost");
  UpdateArmorsNum(chosen);

  if (is_outpost_)
  {
    InitOutpost(chosen);
    active_model_ = ModelType::OUTPOST;
  }
  else
  {
    InitNormal(chosen);
    active_model_ = ModelType::FULL;
  }

  tracker_state_ = State::DETECTING;

  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Tracker Init, mode=%s",
               is_outpost_ ? "outpost" : "normal");
}

void Tracker::Update(const Armors::SharedPtr& armors_msg)
{
  if (jump_cooldown_ > 0)
  {
    --jump_cooldown_;
  }

  if (is_outpost_)
  {
    UpdateOutpost(armors_msg);
  }
  else
  {
    UpdateNormal(armors_msg);
  }
}

// =====================================================================
// 普通模式 Init
// =====================================================================
void Tracker::InitNormal(const Armor& armor)
{
  jump_cooldown_ = 0;
  full_update_count_ = 0;
  full_last_r_ = params_.default_init_radius;
  last_yaw_unwrap_ = 0.0;

  {
    tf2::Quaternion tf_q;
    tf2::fromMsg(armor.pose.orientation, tf_q);
    double roll = NAN, pitch = NAN, yaw = NAN;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    last_yaw_unwrap_ = yaw;
  }
  const double yaw = OrientationToYaw(armor.pose.orientation);

  const double xa = armor.pose.position.x;
  const double ya = armor.pose.position.y;
  const double za = armor.pose.position.z;
  const double r = params_.default_init_radius;
  const double xc = xa + r * std::cos(yaw);
  const double yc = ya + r * std::sin(yaw);
  dz_ = 0.0;
  another_r_ = r;

  // 整车
  Eigen::VectorXd x_full(8);
  x_full << xc, 0, yc, 0, za, yaw, 0, r;

  Eigen::MatrixXd p_full = Eigen::MatrixXd::Zero(8, 8);
  p_full(0, 0) = 0.05;
  p_full(1, 1) = 1.0;
  p_full(2, 2) = 0.05;
  p_full(3, 3) = 1.0;
  p_full(4, 4) = 0.05;
  p_full(5, 5) = 0.1;
  p_full(6, 6) = 2.0;
  p_full(7, 7) = 1.0;
  ekf_full_.SetState(x_full, p_full);

  // 装甲板
  Eigen::VectorXd x_armor(5);
  x_armor << xa, 0, ya, 0, za;

  Eigen::MatrixXd p_armor = Eigen::MatrixXd::Zero(5, 5);
  p_armor(0, 0) = 0.05;
  p_armor(1, 1) = 1.0;
  p_armor(2, 2) = 0.05;
  p_armor(3, 3) = 1.0;
  p_armor(4, 4) = 0.05;
  ekf_armor_.SetState(x_armor, p_armor);
}

// =====================================================================
// 普通模式 Update
// =====================================================================
void Tracker::UpdateNormal(const Armors::SharedPtr& armors_msg)
{
  full_pred_ = ekf_full_.Predict();
  ekf_armor_.Predict();

  predicted_armor_pos_full_ = GetArmorPositionFromFullState(full_pred_);

  bool matched = false;
  bool is_jump = false;

  const auto& armors = armors_msg->armors;
  std::vector<Armor> target_id_armors;
  target_id_armors.reserve(armors.size());
  std::copy_if(armors.begin(), armors.end(), std::back_inserter(target_id_armors),
               [id = tracked_id_](const Armor& a) { return a.number == id; });

  if (target_id_armors.size() != armors.size())
  {
    DoYouWantToChangeTarget(armors_msg);
  }

  if (target_id_armors.empty())
  {
    ClampFullRadius();
    UpdateTrackerState(false);
    SelectActiveModel();
    return;
  }

  double position_diff = DBL_MAX;
  double yaw_diff = DBL_MAX;
  matched =
      MatchArmorFull(target_id_armors, full_pred_, position_diff, yaw_diff, is_jump);
  info_position_diff = position_diff;
  info_yaw_diff = yaw_diff;

  if (matched)
  {
    const double measured_yaw = OrientationToYaw(tracked_armor_.pose.orientation);
    UpdateEkfFull(measured_yaw, tracked_armor_.pose.position);
    UpdateEkfArmor(tracked_armor_.pose.position);
  }
  else if (is_jump)
  {
    if (jump_cooldown_ <= 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Armor Jump!");
      HandleArmorJumpFull(tracked_armor_);
      HandleArmorJumpArmor(tracked_armor_);
      jump_cooldown_ = JUMP_COOLDOWN_FRAMES;
    }
  }
  else
  {
    const double health_rate = ekf_full_.GetHealthRate();
    if (health_rate < 0.2)
    {
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
                  "Full EKF health rate: %f, resetting", health_rate);
      const double yaw_now = OrientationToYaw(tracked_armor_.pose.orientation);
      ResetStateFull(yaw_now, tracked_armor_.pose.position);
    }
  }

  ClampFullRadius();
  UpdateTrackerState(matched);
  SelectActiveModel();
}

// =====================================================================
// 普通模式：多板匹配
// =====================================================================
bool Tracker::MatchArmorFull(const std::vector<Armor>& target_id_armors,
                             const Eigen::VectorXd& full_pred, double& position_diff,
                             double& yaw_diff, bool& is_jump)
{
  const double sign = full_pred(6) >= 0.0 ? 1.0 : -1.0;
  const double a2a_yaw_diff =
      2 * std::numbers::pi_v<double> / static_cast<double>(tracked_armors_num_);
  const double next_yaw = full_pred(5) - sign * a2a_yaw_diff;

  if (target_id_armors.size() == 1)
  {
    const auto& armor = target_id_armors[0];
    const double obs_yaw = OrientationToYaw(armor.pose.orientation);
    const double match_yaw_diff = std::abs(AngleDiff(obs_yaw, full_pred(5)));
    const double jump_yaw_diff = std::abs(AngleDiff(obs_yaw, next_yaw));

    const auto& p = armor.pose.position;
    const Eigen::Vector3d pos_vec(p.x, p.y, p.z);
    const double match_pos_diff = (predicted_armor_pos_full_ - pos_vec).norm();

    if (match_pos_diff < params_.max_match_distance)
    {
      if (match_yaw_diff < params_.max_match_yaw_diff)
      {
        tracked_armor_ = armor;
        is_jump = false;
        position_diff = match_pos_diff;
        yaw_diff = match_yaw_diff;
        return true;
      }
      else if (jump_yaw_diff < params_.max_match_yaw_diff)
      {
        tracked_armor_ = armor;
        is_jump = true;
        position_diff = match_pos_diff;
        yaw_diff = jump_yaw_diff;
        return false;
      }
    }
    else
    {
      tracked_armor_ = armor;
      is_jump = false;
      position_diff = match_pos_diff;
      yaw_diff = match_yaw_diff;
      return false;
    }

    position_diff = match_pos_diff;
    yaw_diff = match_yaw_diff;
  }
  else if (target_id_armors.size() == 2)
  {
    const auto& a0 = target_id_armors[0];
    const auto& a1 = target_id_armors[1];
    const double yaw0 = OrientationToYaw(a0.pose.orientation);
    const double yaw1 = OrientationToYaw(a1.pose.orientation);
    const double yaw_a01 = std::abs(AngleDiff(yaw0, yaw1));

    if (yaw_a01 < a2a_yaw_diff * 0.5 || yaw_a01 > a2a_yaw_diff * 1.5)
    {
      tracked_armor_ = a0;
      is_jump = false;
      return false;
    }

    const double dy0 = std::abs(AngleDiff(yaw0, full_pred(5)));
    const double dy1 = std::abs(AngleDiff(yaw1, full_pred(5)));

    if (dy0 < params_.max_match_yaw_diff)
    {
      tracked_armor_ = a0;
      is_jump = false;
      return true;
    }
    else if (dy1 < params_.max_match_yaw_diff)
    {
      tracked_armor_ = a1;
      is_jump = false;
      return true;
    }
    else
    {
      tracked_armor_ = a0;
      is_jump = false;
      return false;
    }
  }

  tracked_armor_ = target_id_armors[0];
  is_jump = false;
  return false;
}

// =====================================================================
// 整车 EKF Update
// =====================================================================
void Tracker::UpdateEkfFull(double measured_yaw, const geometry_msgs::msg::Point& p)
{
  ++full_update_count_;
  measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
  Eigen::VectorXd x_post = ekf_full_.Update(measurement);

  // 高速旋转：锁定 r
  // if (std::fabs(x_post(6)) > 1.5)
  // {
  //   if (full_update_count_ <= 100)
  //   {
  //     full_last_r_ += x_post(7) / 100.0;
  //   }
  //   else
  //   {
  //     x_post(7) = std::clamp(x_post(7), full_last_r_ - 0.0001, full_last_r_ + 0.0001);
  //     full_last_r_ = x_post(7);
  //     ekf_full_.SetState(x_post);
  //   }
  // }
  // else
  // {
    full_update_count_ = 0;
    x_post(7) = std::clamp(x_post(7), full_last_r_ - 0.001, full_last_r_ + 0.001);
    full_last_r_ = x_post(7);
    ekf_full_.SetState(x_post);
  // }
}

// =====================================================================
// 装甲板 EKF Update
// =====================================================================
void Tracker::UpdateEkfArmor(const geometry_msgs::msg::Point& p)
{
  Eigen::VectorXd z(3);
  z << p.x, p.y, p.z;
  ekf_armor_.Update(z);
}

// =====================================================================
// 跳变处理
// =====================================================================
void Tracker::HandleArmorJumpFull(const Armor& current_armor)
{
  const auto& pos = current_armor.pose.position;
  const double yaw = OrientationToYaw(current_armor.pose.orientation);

  Eigen::VectorXd x = ekf_full_.GetState();
  x(5) = yaw;

  if (tracked_armors_num_ == ArmorsNum::NORMAL_4)
  {
    dz_ = x(4) - pos.z;
    std::swap(x(7), another_r_);
    x(4) = pos.z;
  }

  ekf_full_.SetState(x);
  last_tracked_armor_ = current_armor;
}

void Tracker::HandleArmorJumpArmor(const Armor& current_armor)
{
  const auto& pos = current_armor.pose.position;
  Eigen::VectorXd x = ekf_armor_.GetState();
  x(0) = pos.x;
  x(2) = pos.y;
  x(4) = pos.z;
  // 速度 x(1), x(3) 保留 —— 同一辆车的世界系平移速度未变
  ekf_armor_.SetState(x);  // 不传 P，协方差保留
}

// =====================================================================
// 半径钳制（仅整车）
// =====================================================================
void Tracker::ClampFullRadius()
{
  Eigen::VectorXd x = ekf_full_.GetState();
  x(7) = std::clamp(x(7), params_.radius_min, params_.radius_max);
  ekf_full_.SetState(x);
}

// =====================================================================
// 切换跟踪 ID
// =====================================================================
void Tracker::DoYouWantToChangeTarget(const Armors::SharedPtr& armors_msg)
{
  if (armors_msg->armors.empty())
  {
    return;
  }

  double min_distance = DBL_MAX;
  Armor closest = armors_msg->armors[0];
  for (const auto& a : armors_msg->armors)
  {
    if (a.distance_to_image_center < min_distance)
    {
      min_distance = a.distance_to_image_center;
      closest = a;
    }
  }

  if (closest.number != tracked_id_ && closest.number == last_closest_id_)
  {
    if (change_count_ < change_thres_)
    {
      ++change_count_;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
                  "Confirmed target change to ID: %s", closest.number.c_str());
      tracked_id_ = closest.number;
      tracked_armor_ = closest;
      is_outpost_ = (closest.number == "outpost");
      UpdateArmorsNum(closest);

      if (is_outpost_)
      {
        InitOutpost(closest);
        active_model_ = ModelType::OUTPOST;
      }
      else
      {
        InitNormal(closest);
        active_model_ = ModelType::FULL;
      }
      tracker_state_ = State::DETECTING;
      change_count_ = 0;
    }
  }
  else
  {
    change_count_ = 0;
  }

  last_closest_id_ = closest.number;
}

// =====================================================================
// 整车模型重置（健康率过低时）
// =====================================================================
void Tracker::ResetStateFull(double yaw, const geometry_msgs::msg::Point& p)
{
  Eigen::VectorXd x = ekf_full_.GetState();
  const double r = x(7);

  x(0) = p.x + r * std::cos(yaw);
  x(2) = p.y + r * std::sin(yaw);
  x(4) = p.z;
  x(5) = yaw;
  x(1) = 0.0;
  x(3) = 0.0;
  x(6) = 0.0;

  Eigen::MatrixXd p_reset = Eigen::MatrixXd::Zero(8, 8);
  p_reset(0, 0) = 0.03;
  p_reset(1, 1) = 2.0;
  p_reset(2, 2) = 0.03;
  p_reset(3, 3) = 2.0;
  p_reset(4, 4) = 0.03;
  p_reset(5, 5) = 0.05;
  p_reset(6, 6) = 4.0;
  p_reset(7, 7) = 0.2;

  ekf_full_.SetState(x, p_reset);

  full_update_count_ = 0;
  full_last_r_ = r;
}

// =====================================================================
// 选择对外模型（迟滞）
// =====================================================================
void Tracker::SelectActiveModel()
{
  if (is_outpost_)
  {
    active_model_ = ModelType::OUTPOST;
    FillOutputFromOutpost();
    return;
  }

  const Eigen::VectorXd x_full = ekf_full_.GetState();
  const double abs_v_yaw = std::fabs(x_full(6));

  if (active_model_ == ModelType::FULL)
  {
    if (abs_v_yaw < params_.v_yaw_armor_threshold)
    {
      active_model_ = ModelType::ARMOR;
    }
  }
  else  // ARMOR
  {
    if (abs_v_yaw > params_.v_yaw_full_threshold)
    {
      active_model_ = ModelType::FULL;
    }
  }

  FillOutputFromFull(active_model_ == ModelType::FULL);
}

// =====================================================================
// 输出填充
// =====================================================================
void Tracker::FillOutputFromFull(bool is_center)
{
  const Eigen::VectorXd xf = ekf_full_.GetState();
  output_.armor_type = tracked_armor_.type;
  output_.armor_number = tracked_armor_.number;
  output_.armors_num = static_cast<int>(tracked_armors_num_);
  output_.outpost_idx = 0;
  output_.dz = dz_;
  output_.radius_1 = xf(7);
  output_.radius_2 = another_r_;
  output_.yaw = xf(5);
  output_.v_yaw = xf(6);

  if (is_center)
  {
    output_.is_center = true;
    output_.position = Eigen::Vector3d(xf(0), xf(2), xf(4));
    output_.velocity = Eigen::Vector3d(xf(1), xf(3), 0.0);
  }
  else
  {
    const Eigen::VectorXd xa = ekf_armor_.GetState();
    output_.is_center = false;
    output_.position = Eigen::Vector3d(xa(0), xa(2), xa(4));
    output_.velocity = Eigen::Vector3d(xa(1), xa(3), 0.0);
  }
}

void Tracker::FillOutputFromOutpost()
{
  const Eigen::VectorXd xo = ekf_outpost_.GetState();
  output_.is_center = true;
  output_.armor_type = tracked_armor_.type;
  output_.armor_number = tracked_armor_.number;
  output_.armors_num = static_cast<int>(tracked_armors_num_);
  output_.outpost_idx = outpost_idx_;
  output_.position = Eigen::Vector3d(xo(0), xo(1), xo(2));
  output_.velocity = Eigen::Vector3d(0, 0, 0);
  output_.yaw = xo(3);
  output_.v_yaw = xo(4);
  output_.radius_1 = params_.outpost_r;
  output_.radius_2 = params_.outpost_r;
  output_.dz = params_.outpost_dz;
}

// =====================================================================
// 前哨站 Init
// =====================================================================
void Tracker::InitOutpost(const Armor& armor)
{
  jump_cooldown_ = 0;
  outpost_update_count_ = 0;
  outpost_v_yaw_avg_ = 0.0;
  outpost_z_avg_ = 0.0;
  outpost_motion_ = OutpostMotion::LEARNING;
  outpost_v_yaw_sign_ = 1;
  outpost_idx_ = 1;  // 默认从中间档开始
  last_yaw_unwrap_ = 0.0;

  {
    tf2::Quaternion tf_q;
    tf2::fromMsg(armor.pose.orientation, tf_q);
    double roll = NAN, pitch = NAN, yaw = NAN;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    last_yaw_unwrap_ = yaw;
  }
  const double yaw = OrientationToYaw(armor.pose.orientation);

  const double xa = armor.pose.position.x;
  const double ya = armor.pose.position.y;
  const double za = armor.pose.position.z;
  const double r = params_.outpost_r;

  // 默认 idx=1（中间），所以 zc = za_armor
  const double zc = za;
  const double xc = xa + r * std::cos(yaw);
  const double yc = ya + r * std::sin(yaw);

  Eigen::VectorXd x(5);
  x << xc, yc, zc, yaw, 0;

  Eigen::MatrixXd p0 = Eigen::MatrixXd::Zero(5, 5);
  p0(0, 0) = 0.05;
  p0(1, 1) = 0.05;
  p0(2, 2) = 0.05;
  p0(3, 3) = 0.1;
  p0(4, 4) = 2.0;

  ekf_outpost_.SetState(x, p0);
  last_tracked_armor_ = armor;
}

// =====================================================================
// 前哨站 Update
// =====================================================================
void Tracker::UpdateOutpost(const Armors::SharedPtr& armors_msg)
{
  Eigen::VectorXd outpost_pred = ekf_outpost_.Predict();

  bool matched = false;
  bool is_jump = false;

  const auto& armors = armors_msg->armors;
  std::vector<Armor> target_id_armors;
  target_id_armors.reserve(armors.size());
  std::copy_if(armors.begin(), armors.end(), std::back_inserter(target_id_armors),
               [id = tracked_id_](const Armor& a) { return a.number == id; });

  if (target_id_armors.size() != armors.size())
  {
    DoYouWantToChangeTarget(armors_msg);
  }

  if (target_id_armors.empty())
  {
    UpdateTrackerState(false);
    FillOutputFromOutpost();
    return;
  }

  double position_diff = DBL_MAX;
  double yaw_diff = DBL_MAX;
  matched =
      MatchArmorOutpost(target_id_armors, outpost_pred, position_diff, yaw_diff, is_jump);
  info_position_diff = position_diff;
  info_yaw_diff = yaw_diff;

  if (matched)
  {
    const double measured_yaw = OrientationToYaw(tracked_armor_.pose.orientation);
    UpdateOutpostIdx(tracked_armor_.pose.position, false);
    UpdateEkfOutpost(measured_yaw, tracked_armor_.pose.position);
    last_tracked_armor_ = tracked_armor_;
  }
  else if (is_jump)
  {
    if (jump_cooldown_ <= 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Outpost armor jump!");
      UpdateOutpostIdx(tracked_armor_.pose.position, true);
      HandleArmorJumpOutpost(tracked_armor_);
      last_tracked_armor_ = tracked_armor_;
      jump_cooldown_ = JUMP_COOLDOWN_FRAMES;
    }
  }

  ApplyOutpostMotionLogic();
  UpdateTrackerState(matched);
  FillOutputFromOutpost();
}

// =====================================================================
// 前哨站匹配
// =====================================================================
bool Tracker::MatchArmorOutpost(const std::vector<Armor>& target_id_armors,
                                const Eigen::VectorXd& outpost_pred,
                                double& position_diff, double& yaw_diff, bool& is_jump)
{
  const double pred_yaw = outpost_pred(3);
  const double pred_v_yaw = outpost_pred(4);
  const double sign = pred_v_yaw >= 0.0 ? 1.0 : -1.0;
  const double a2a_yaw_diff =
      2 * std::numbers::pi_v<double> / static_cast<double>(tracked_armors_num_);
  const double next_yaw = pred_yaw - sign * a2a_yaw_diff;

  const Eigen::Vector3d pred_armor_pos = GetArmorPositionFromOutpostState(outpost_pred);

  double best_yaw_diff = DBL_MAX;
  double best_jump_diff = DBL_MAX;
  size_t best_idx = 0;
  size_t best_jump_idx = 0;

  for (size_t i = 0; i < target_id_armors.size(); ++i)
  {
    const double obs_yaw = OrientationToYaw(target_id_armors[i].pose.orientation);
    const double dy = std::abs(AngleDiff(obs_yaw, pred_yaw));
    const double djy = std::abs(AngleDiff(obs_yaw, next_yaw));
    if (dy < best_yaw_diff)
    {
      best_yaw_diff = dy;
      best_idx = i;
    }
    if (djy < best_jump_diff)
    {
      best_jump_diff = djy;
      best_jump_idx = i;
    }
  }

  const auto& cand = target_id_armors[best_idx];
  const auto& pos = cand.pose.position;
  const Eigen::Vector3d pos_vec(pos.x, pos.y, pos.z);
  const double pos_diff = (pred_armor_pos - pos_vec).norm();

  if (best_yaw_diff < params_.max_match_yaw_diff && pos_diff < params_.max_match_distance)
  {
    tracked_armor_ = cand;
    is_jump = false;
    position_diff = pos_diff;
    yaw_diff = best_yaw_diff;
    return true;
  }

  if (best_jump_diff < params_.max_match_yaw_diff)
  {
    tracked_armor_ = target_id_armors[best_jump_idx];
    is_jump = true;
    position_diff = pos_diff;
    yaw_diff = best_jump_diff;
    return false;
  }

  tracked_armor_ = cand;
  is_jump = false;
  position_diff = pos_diff;
  yaw_diff = best_yaw_diff;
  return false;
}

// =====================================================================
// 前哨站 EKF Update（送入前先把 z 修正到 zc 等价）
// =====================================================================
void Tracker::UpdateEkfOutpost(double measured_yaw, const geometry_msgs::msg::Point& p)
{
  ++outpost_update_count_;
  const double z_corrected =
      p.z + (1.0 - static_cast<double>(outpost_idx_)) * params_.outpost_dz;

  measurement = Eigen::Vector4d(p.x, p.y, z_corrected, measured_yaw);
  ekf_outpost_.Update(measurement);
}

// =====================================================================
// 前哨站跳变：替换 yaw
// =====================================================================
void Tracker::HandleArmorJumpOutpost(const Armor& current_armor)
{
  const double yaw = OrientationToYaw(current_armor.pose.orientation);
  Eigen::VectorXd x = ekf_outpost_.GetState();
  x(3) = yaw;
  ekf_outpost_.SetState(x);
}

// =====================================================================
// 前哨站 旋转/静止 判定 + v_yaw / zc 钳制
// =====================================================================
void Tracker::ApplyOutpostMotionLogic()
{
  Eigen::VectorXd x = ekf_outpost_.GetState();

  if (outpost_motion_ == OutpostMotion::LEARNING)
  {
    if (outpost_update_count_ <= params_.outpost_learning_frames &&
        outpost_update_count_ > 0)
    {
      const double n = static_cast<double>(params_.outpost_learning_frames);
      outpost_v_yaw_avg_ += x(4) / n;
      outpost_z_avg_ += x(2) / n;
    }

    if (outpost_update_count_ > params_.outpost_learning_frames)
    {
      if (std::fabs(outpost_v_yaw_avg_) > params_.outpost_static_threshold)
      {
        outpost_motion_ = OutpostMotion::ROTATING;
        outpost_v_yaw_sign_ = (outpost_v_yaw_avg_ >= 0.0) ? 1 : -1;
        RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
                    "Outpost decided ROTATING, sign=%d", outpost_v_yaw_sign_);
      }
      else
      {
        outpost_motion_ = OutpostMotion::STATIC;
        outpost_v_yaw_sign_ = (x(4) >= 0.0) ? 1 : -1;
        RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Outpost decided STATIC");
      }
    }
    return;
  }

  // 锁定后：钳制 v_yaw 和 zc
  if (outpost_motion_ == OutpostMotion::ROTATING)
  {
    x(4) = static_cast<double>(outpost_v_yaw_sign_) * params_.outpost_vyaw_abs;
  }
  else  // STATIC
  {
    x(4) = 0.0;
  }
  //x(2) = std::clamp(x(2), outpost_z_avg_ - 0.001, outpost_z_avg_ + 0.001);
  ekf_outpost_.SetState(x);
}

// =====================================================================
// 状态机
// =====================================================================
void Tracker::UpdateTrackerState(bool matched)
{
  if (tracker_state_ == State::DETECTING)
  {
    if (matched)
    {
      ++detect_count_;
      if (detect_count_ > params_.tracking_thres)
      {
        detect_count_ = 0;
        tracker_state_ = State::TRACKING;
      }
    }
    else
    {
      detect_count_ = 0;
      tracker_state_ = State::LOST;
    }
  }
  else if (tracker_state_ == State::TRACKING)
  {
    if (!matched)
    {
      tracker_state_ = State::TEMP_LOST;
      ++lost_count_;
    }
  }
  else if (tracker_state_ == State::TEMP_LOST)
  {
    if (!matched)
    {
      ++lost_count_;
      if (lost_count_ > lost_thres_)
      {
        lost_count_ = 0;
        tracker_state_ = State::LOST;
      }
    }
    else
    {
      tracker_state_ = State::TRACKING;
      lost_count_ = 0;
    }
  }
}

void Tracker::UpdateArmorsNum(const Armor& armor)
{
  tracked_armors_num_ =
      (armor.number == "outpost") ? ArmorsNum::OUTPOST_3 : ArmorsNum::NORMAL_4;
}

// =====================================================================
// 共用工具
// =====================================================================
double Tracker::OrientationToYaw(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll = NAN, pitch = NAN, yaw = NAN;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  yaw = last_yaw_unwrap_ + angles::shortest_angular_distance(last_yaw_unwrap_, yaw);
  last_yaw_unwrap_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::GetArmorPositionFromFullState(const Eigen::VectorXd& x) const
{
  // [xc, v_xc, yc, v_yc, za, yaw, v_yaw, r]
  const double xc = x(0), yc = x(2), za = x(4), yaw = x(5), r = x(7);
  return Eigen::Vector3d(xc - r * std::cos(yaw), yc - r * std::sin(yaw), za);
}

Eigen::Vector3d Tracker::GetArmorPositionFromOutpostState(const Eigen::VectorXd& x) const
{
  // [xc, yc, zc, yaw, v_yaw]
  const double xc = x(0), yc = x(1), zc = x(2), yaw = x(3);
  const double r = params_.outpost_r;
  // 当前 idx 对应的板：za = zc - (1 - idx) * dz
  const double za = zc - (1.0 - static_cast<double>(outpost_idx_)) * params_.outpost_dz;
  return Eigen::Vector3d(xc - r * std::cos(yaw), yc - r * std::sin(yaw), za);
}

}  // namespace rm_auto_aim
