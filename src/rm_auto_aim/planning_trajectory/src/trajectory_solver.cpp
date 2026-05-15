#include "planning_trajectory/trajectory_solver.hpp"

#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>

namespace rm_auto_aim
{

TrajectorySolver::TrajectorySolver(const double& k, const double& bias_time,
                                   const double& s_bias, const double& z_bias,
                                   const double& pitch_bias, CalculateMode calculate_mode,
                                   const Table::TableConfig& table_config,
                                   const Table::TableConfig& table_config_lob_)
    : table_(std::make_shared<Table>(table_config)),
      table_lob_(std::make_shared<Table>(table_config_lob_)),
      calculate_mode_(calculate_mode),
      k_(k),
      pitch_bias_(pitch_bias),
      bias_time_(bias_time),
      s_bias_(s_bias),
      z_bias_(z_bias)
{
  if (calculate_mode_ == CalculateMode::TABLE_LOOKUP)
  {
    table_->Init();
    table_lob_->Init();
    current_table_ = table_;
    if (current_table_->IsInit() && table_lob_->IsInit())
    {
      RCLCPP_INFO(logger_, "Trajectory table initialized successfully");
    }
    else if (!table_->IsInit())
    {
      calculate_mode_ = CalculateMode::NORMAL;
      RCLCPP_WARN(logger_, "Using normal calculation mode");
    }
    else
    {
      RCLCPP_WARN(logger_,
                  "LOB table failed to initialize, LOB mode will be unavailable");
    }
  }
}

void TrajectorySolver::Init(
    const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
{
  if (!std::isnan(velocity_msg->velocity))
  {
    current_v_ = velocity_msg->velocity;
    RCLCPP_DEBUG(logger_, "Velocity updated: %.2f m/s", current_v_);
  }
  else
  {
    RCLCPP_WARN(logger_, "Invalid velocity, using default: 20.0 m/s");
    current_v_ = 12.0f;
  }
}

void TrajectorySolver::ReBuild()
{
  selected_idx_ = LOST;
  choose_next_ = false;
  should_last_shot_ = false;
  turn_s_ = 0.0;
  step_s_ = 0.0;
  start_turn_ = time_point::min();
  end_turn_ = time_point::min();
  last_start_turn_ = time_point::min();
  pre_center_ = {};
  pre_position_.fill({});
  last_x_v_ = 0.0;
  last_y_v_ = 0.0;
  last_v_yaw_ = 0.0;
  last_pitch_ = 0.0;
  last_yaw_ = 0.0;
  fly_time_ = 0.0;
}

TrajectorySolver::TarPostion TrajectorySolver::PredictCenter(double time_delay)
{
  TarPostion center;
  if (target_.num == 4)
  {
    center.x = target_.position.x + target_.velocity.x * time_delay;
    center.y = target_.position.y + target_.velocity.y * time_delay;
    center.z = target_.position.z;
    center.yaw = NormalizeAngle(target_.position.yaw + target_.velocity.yaw * time_delay);
  }
  else
  {
    center.x = target_.position.x;
    center.y = target_.position.y;
    center.z = target_.position.z;
    center.yaw = NormalizeAngle(target_.position.yaw + target_.velocity.yaw * time_delay);
  }
  return center;
}

TrajectorySolver::TarPostion TrajectorySolver::PredictArmor(
    int idx, const TrajectorySolver::TarPostion& pre_center)
{
  TarPostion pre_pos;
  const double sign = target_.velocity.yaw >= 0.0 ? 1.0 : -1.0;
  const double delta = sign * idx * 2.0 * std::numbers::pi_v<double> / target_.num;
  const double tmp_yaw = NormalizeAngle(pre_center.yaw - delta);

  if (target_.num == 4)
  {
    const double radius = (idx % 2 == 0) ? target_.radius1 : target_.radius2;
    pre_pos.x = pre_center.x - radius * std::cos(tmp_yaw);
    pre_pos.y = pre_center.y - radius * std::sin(tmp_yaw);
    pre_pos.z = pre_center.z;
    pre_pos.yaw = tmp_yaw;
  }
  else
  {
    const double radius = target_.radius1;
    pre_pos.x = pre_center.x - radius * std::cos(tmp_yaw);
    pre_pos.y = pre_center.y - radius * std::sin(tmp_yaw);

    const int offset = (sign > 0.0) ? idx : ((target_.num - idx) % target_.num);
    const int id = (target_.outpost_idx + offset) % target_.num;

    pre_pos.z = pre_center.z + outpost_dz * (id - 1);
    pre_pos.yaw = tmp_yaw;
  }

  return pre_pos;
}

// 从图片时间到打到的时间：自瞄处理的时间+电控延迟(从视觉发信号到电机动和发弹延迟)+云台转动时间+飞行时间
// msg消息的频率即我们发送开火指令的频率，这可以作为我们的步长时间
void TrajectorySolver::PredictAllArmorPosition(double time_delay)
{
  pre_center_ = PredictCenter(time_delay);
  pre_position_.fill({});

  for (int i = 0; i < target_.num; ++i)
  {
    pre_position_[i] = PredictArmor(i, pre_center_);
  }
}

void TrajectorySolver::PredictOneArmorPosition(double time_delay, int idx)
{
  pre_center_ = PredictCenter(time_delay);
  pre_position_[idx] = PredictArmor(idx, pre_center_);
}

// 计算简化单向空气阻力模型下的弹道高度，用于正常模式
double TrajectorySolver::MonoDirectionalAirResistanceModel(double s, double angle,
                                                           double v)
{
  double cos_angle = std::cos(angle);
  if (cos_angle <= 0)
  {
    RCLCPP_WARN(logger_, "Invalid angle: cos(angle) <= 0");
    fly_time_ = 0;
    return 0;
  }

  fly_time_ = (std::exp(k_ * s) - 1) / (k_ * v * cos_angle);

  if (fly_time_ < 0)
  {
    RCLCPP_WARN(logger_, "Exceeding maximum range! s: %.2f, v: %.2f", s, v);
    fly_time_ = 0;
    return 0;
  }

  return v * sin(angle) * fly_time_ - GRAVITY * fly_time_ * fly_time_ / 2;
}

// 计算俯仰角(两种模式)
// 计算俯仰角(两种模式)
double TrajectorySolver::SolvePitch(double x, double y, double z)
{
  // 计算水平距离
  double distance = std::sqrt(x * x + y * y);
  double target_s = distance + s_bias_;
  double target_z = z + z_bias_;
  if (std::fabs(z) < 0.01)
  {
    RCLCPP_WARN(logger_, "Target z is too low: %.2f", target_z);
    return 0.0f;
  }

  double pitch = 0.0f;
  bool use_iteration = false;  // 是否需要走迭代法

  if (calculate_mode_ == CalculateMode::TABLE_LOOKUP && current_table_->IsInit())
  {
    // 查表法
    auto res = current_table_->Check(target_s, target_z);
    if (!std::isnan(res.pitch))
    {
      fly_time_ = res.t;
      pitch = res.pitch;
    }
    else
    {
      // 查表结果为 NaN，回退到迭代法
      RCLCPP_WARN(logger_, "Table lookup nan for s: %.2f, z: %.2f, fallback to iteration",
                  target_s, target_z);
      use_iteration = true;
    }
  }
  else
  {
    // 非查表模式，直接使用迭代法
    use_iteration = true;
  }

  if (use_iteration)
  {
    // 正常模式下的迭代计算
    double z_temp = target_z;
    bool converged = false;

    for (int i = 0; i < 20; ++i)
    {
      if (std::isnan(z_temp))
      {
        RCLCPP_ERROR(logger_, "z_temp is NaN during iteration");
        return 0.0f;
      }

      pitch = std::atan2(z_temp, target_s);
      double z_actual = MonoDirectionalAirResistanceModel(target_s, pitch, current_v_);
      double dz = 0.3f * (target_z - z_actual);
      z_temp += dz;

      if (fabs(dz) < 1e-5f)
      {
        RCLCPP_DEBUG(logger_, "Pitch convergence after %d iterations", i + 1);
        converged = true;
        break;
      }
    }

    if (!converged)
    {
      RCLCPP_WARN(logger_, "Pitch iteration did not converge within 20 steps");
    }
  }

  pitch += pitch_bias_;
  return pitch;
}

double TrajectorySolver::SolveYaw(double x, double y) const { return std::atan2(y, x); }

double fast_atan(double x, double y)
{
  double x_y = y / x;
  double x_y_2 = x_y * x_y;
  return x_y * (0.99997726f + x_y_2 * (-0.33262347f + x_y_2 * 0.19354346f));
}

std::pair<double, double> TrajectorySolver::ComputeFireYawWindow(
    const TarPostion& armor) const
{
  const double half_length_ =
      ((target_.type == "small") ? SMALL_HALF_LENGTH : LARGE_HALF_LENGTH) - 0.03;

  const double sy = std::sin(armor.yaw);
  const double cy = std::cos(armor.yaw);

  const double ax = armor.x - half_length_ * sy;
  const double ay = armor.y + half_length_ * cy;
  const double bx = armor.x + half_length_ * sy;
  const double by = armor.y - half_length_ * cy;

  const double angle_c = SolveYaw(armor.x, armor.y);
  const double angle_a = SolveYaw(ax, ay);
  const double angle_b = SolveYaw(bx, by);

  const double lo = AngleDiff(angle_b, angle_c);
  const double hi = AngleDiff(angle_a, angle_c);
  return {std::min(lo, hi), std::max(lo, hi)};
}

// 快速打击符号fast_fire为false时，只打云台和跟踪都就位的装甲板
bool TrajectorySolver::CanFire(double tar_yaw, double tar_pitch, bool is_fast_fire)
{
  if (!HasValidSelection())
  {
    return false;
  }

  const auto& p = pre_position_[selected_idx_];
  const auto [yaw_lo, yaw_hi] = ComputeFireYawWindow(p);

  const double control_delta = AngleDiff(tar_yaw, gimbal_yaw_);
  const bool yaw_ok = (control_delta >= yaw_lo) && (control_delta <= yaw_hi);
  const bool pitch_ok = std::fabs(tar_pitch - gimbal_pitch_) <= 0.02;

  const bool stable_tracking = std::fabs(target_.velocity.x - last_x_v_) < 0.4 &&
                               std::fabs(target_.velocity.y - last_y_v_) < 0.3 &&
                               std::fabs(target_.velocity.yaw - last_v_yaw_) < 0.3;

  if (!stable_tracking && !is_fast_fire && !should_last_shot_)
  {
    return false;
  }

  const bool angle_diff_exceeds = !yaw_ok || !pitch_ok;

  if (choose_next_)
  {
    if (angle_diff_exceeds)
    {
      return false;
    }
    return is_fast_fire;
  }
  if (angle_diff_exceeds)
  {
    return is_fast_fire;
  }
  return true;
}

void TrajectorySolver::GlobalSelectArmor(double time_delay)
{
  double best_cost = std::numeric_limits<double>::infinity();
  int best_idx = 0;
  double best_time = time_delay;

  PredictAllArmorPosition(time_delay);

  for (int i = 0; i < target_.num; ++i)
  {
    const double base_yaw = SolveYaw(pre_position_[i].x, pre_position_[i].y);
    const double turn_time = 0.05 * std::fabs(AngleDiff(base_yaw, gimbal_yaw_));

    const TarPostion center = PredictCenter(time_delay + turn_time);
    const TarPostion armor = PredictArmor(i, center);

    const double aim_yaw = SolveYaw(armor.x, armor.y);
    const double cost = std::fabs(AngleDiff(aim_yaw, gimbal_yaw_));

    if (cost < best_cost)
    {
      best_cost = cost;
      best_idx = i;
      best_time = time_delay + turn_time;
    }
  }

  selected_idx_ = best_idx;
}

void TrajectorySolver::LocalSelectArmor(double time_delay)
{
  const TarPostion center0 = PredictCenter(time_delay);
  const TarPostion armor0 = PredictArmor(1, center0);
  const double center_yaw_0 = SolveYaw(center0.x, center0.y);
  const double armor_yaw_err_0 =
      std::fabs(AngleDiff(SolveYaw(armor0.x, armor0.y), center_yaw_0));
  const double s_0 = armor0.x * armor0.x + armor0.y * armor0.y;

  const double t1 = time_delay + 0.01 * std::fabs(target_.velocity.yaw);
  const TarPostion center1 = PredictCenter(t1);
  const TarPostion armor1 = PredictArmor(1, center1);
  const double center_yaw_1 = SolveYaw(center1.x, center1.y);
  const double armor_yaw_err_1 =
      std::fabs(AngleDiff(SolveYaw(armor1.x, armor1.y), center_yaw_1));
  const double s_1 = armor1.x * armor1.x + armor1.y * armor1.y;

  choose_next_ = (armor_yaw_err_1 <= armor_yaw_err_0) && (s_1 <= s_0);
  if (choose_next_)
  {
    choose_next_count_++;
  }
  else
  {
    choose_next_count_ = 0;
  }

  if (choose_next_count_ > 1)
  {
    choose_next_ = 1;
  }
  else
  {
    choose_next_ = 0;
  }

  if (last_outpost_idx_ == target_.outpost_idx && target_.num == 3)
  {
    selected_idx_ = (selected_idx_ == 1 ? 1 : (choose_next_ ? 1 : 0));
  }
  else
  {
    selected_idx_ = (choose_next_ ? 1 : 0);
  }
}

void TrajectorySolver::PreSelectArmor(double time_delay)
{
  const int current_idx = selected_idx_;
  const int next_idx = (current_idx + 1) % target_.num;

  const double pre_time_delay = time_delay + 2.0 * bias_time_;

  const TarPostion center0 = PredictCenter(pre_time_delay);
  const TarPostion armor0 = PredictArmor(current_idx, center0);
  const double center_yaw_0 = SolveYaw(center0.x, center0.y);
  const double armor_yaw_err_0 =
      std::fabs(AngleDiff(SolveYaw(armor0.x, armor0.y), center_yaw_0));
  const double s_0 = armor0.x * armor0.x + armor0.y * armor0.y;

  const TarPostion center1 = PredictCenter(pre_time_delay + turn_s_);
  const TarPostion armor1 = PredictArmor(next_idx, center1);
  const double center_yaw_1 = SolveYaw(center1.x, center1.y);
  const double armor_yaw_err_1 =
      std::fabs(AngleDiff(SolveYaw(armor1.x, armor1.y), center_yaw_1));
  const double s_1 = armor1.x * armor1.x + armor1.y * armor1.y;

  const bool pre_turn = (armor_yaw_err_1 <= armor_yaw_err_0) && (s_1 <= s_0);

  should_last_shot_ = !(!choose_next_ && pre_turn);
}

void TrajectorySolver::AutoSelectArmor(double time_delay, bool is_pre_select)
{
  if (selected_idx_ == LOST)
  {
    GlobalSelectArmor(time_delay);
  }
  else
  {
    LocalSelectArmor(time_delay);
  }

  if (is_pre_select)
  {
    PreSelectArmor(time_delay);
  }
  else
  {
    should_last_shot_ = true;
  }
}

// 装甲板 CV 模型分支：上游给的就是装甲板位置/速度
// 不建模(不从中心反推)、不择板(固定 idx=0)
void TrajectorySolver::SolveByArmorCV(double time_delay)
{
  selected_idx_ = 0;
  choose_next_ = false;
  should_last_shot_ = true;

  pre_center_ = {};
  pre_position_.fill({});

  pre_position_[0].x = target_.position.x + target_.velocity.x * time_delay;
  pre_position_[0].y = target_.position.y + target_.velocity.y * time_delay;
  pre_position_[0].z = target_.position.z + target_.velocity.z * time_delay;
  pre_position_[0].yaw =
      NormalizeAngle(target_.position.yaw + target_.velocity.yaw * time_delay);
}

void TrajectorySolver::UpdateFireLogicMode()
{
  if (choose_next_ && !last_choose_next_)
  {
    start_turn_ = std::chrono::steady_clock::now();
  }
  else if (!choose_next_ && last_choose_next_)
  {
    end_turn_ = std::chrono::steady_clock::now();
  }

  const bool has_complete_cycle =
      (end_turn_ != time_point::min() && start_turn_ != time_point::min() &&
       last_start_turn_ != time_point::min());

  if (has_complete_cycle)
  {
    turn_s_ = std::chrono::duration<double>(end_turn_ - start_turn_).count();
    step_s_ = std::chrono::duration<double>(start_turn_ - last_start_turn_).count();

    if (step_s_ > 1e-6)
    {
      const double ratio = turn_s_ / step_s_;

      if (fire_logic_mode_ == FireLogicMode::COMMON)
      {
        if (ratio >= 0.99)
        {
          fire_logic_mode_ = FireLogicMode::SPIN_TEMP;
        }
      }
      else if (fire_logic_mode_ == FireLogicMode::SPIN_TEMP)
      {
        if (ratio < 0.99 - 0.05)
        {
          fire_logic_mode_ = FireLogicMode::COMMON;
        }
        else if (ratio >= 0.99)
        {
          fire_logic_mode_ = FireLogicMode::SPIN;
        }
      }
      else if (fire_logic_mode_ == FireLogicMode::SPIN)
      {
        if (ratio < 0.99 - 0.05)
        {
          fire_logic_mode_ = FireLogicMode::COMMON;
        }
      }
    }

    last_start_turn_ = start_turn_;
    start_turn_ = time_point::min();
    end_turn_ = time_point::min();
  }
  else if (choose_next_ && !last_choose_next_)
  {
    last_start_turn_ = start_turn_;
  }

  if (HasValidSelection())
  {
    last_selected_idx_for_turn_ = selected_idx_;
  }
}

void TrajectorySolver::UpdateSolveState(double& pitch, double& yaw, bool& is_fire,
                                        double& aim_x, double& aim_y, double& aim_z,
                                        int& idx)
{
  idx = selected_idx_;

  if (!HasValidSelection())
  {
    aim_x = 0.0;
    aim_y = 0.0;
    aim_z = 0.0;
    pitch = last_pitch_;
    yaw = last_yaw_;
    is_fire = false;
    idx = LOST;
    return;
  }

  aim_x = pre_position_[selected_idx_].x;
  aim_y = pre_position_[selected_idx_].y;
  aim_z = pre_position_[selected_idx_].z;

  // aim_x = pre_position_[0].x;
  // aim_y = pre_position_[0].y;
  // aim_z = pre_position_[0].z;

  pitch = SolvePitch(aim_x, aim_y, aim_z);

  if (fire_logic_mode_ == FireLogicMode::SPIN)
  {
    yaw = SolveYaw(pre_center_.x, pre_center_.y);

    const double aim_yaw = SolveYaw(aim_x, aim_y);
    is_fire = std::fabs(AngleDiff(aim_yaw, gimbal_yaw_)) <
              0.013;  // CanFire(gimbal_yaw_, pitch, false);
    if (is_fire)
    {
      yaw = aim_yaw;
    }
  }
  else
  {
    yaw = SolveYaw(aim_x, aim_y);
    is_fire = CanFire(yaw, pitch, true);
  }

  last_pitch_ = pitch;
  last_yaw_ = yaw;
  last_x_v_ = target_.velocity.x;
  last_y_v_ = target_.velocity.y;
  last_v_yaw_ = target_.velocity.yaw;
  last_choose_next_ = choose_next_;
  last_outpost_idx_ = target_.outpost_idx;
}

bool TrajectorySolver::IsFarSpinningOutpost() const
{
  const double distance = std::hypot(target_.position.x, target_.position.y);

  return target_.num == 3 && distance > FAR_OUTPOST_DISTANCE &&
         std::fabs(target_.velocity.yaw) > OUTPOST_SPIN_VYAW;
}

int TrajectorySolver::SelectOutpostBottomArmor() const
{
  const double sign = target_.velocity.yaw >= 0.0 ? 1.0 : -1.0;

  for (int i = 0; i < target_.num; ++i)
  {
    const int offset = sign > 0.0 ? i : ((target_.num - i) % target_.num);
    const int id = (target_.outpost_idx + offset) % target_.num;

    // PredictArmor() 里：
    // z = center.z + outpost_dz * (id - 1)
    // 所以 id == 0 是底板
    if (id == 0)
    {
      return i;
    }
  }

  return 0;
}

void TrajectorySolver::SolveFarOutpostBottom(double send_time, double& pitch, double& yaw,
                                             bool& is_fire, double& aim_x, double& aim_y,
                                             double& aim_z, int& idx)
{
  selected_idx_ = SelectOutpostBottomArmor();
  choose_next_ = false;
  should_last_shot_ = true;

  double time_delay = fly_time_ + bias_time_ + send_time;

  PredictOneArmorPosition(time_delay, selected_idx_);

  SolvePitch(pre_position_[selected_idx_].x, pre_position_[selected_idx_].y,
             pre_position_[selected_idx_].z);

  time_delay = fly_time_ + bias_time_ + send_time;

  PredictOneArmorPosition(time_delay, selected_idx_);

  aim_x = pre_position_[selected_idx_].x;
  aim_y = pre_position_[selected_idx_].y;
  aim_z = pre_position_[selected_idx_].z;
  idx = selected_idx_;

  pitch = SolvePitch(aim_x, aim_y, aim_z);

  const double aim_yaw = SolveYaw(aim_x, aim_y);

  // 关键：云台停止转动，不追随前哨站
  yaw = gimbal_yaw_;

  const double yaw_delta = AngleDiff(aim_yaw, gimbal_yaw_);

  const auto [yaw_lo, yaw_hi] = ComputeFireYawWindow(pre_position_[selected_idx_]);

  const bool yaw_ok = yaw_delta >= yaw_lo && yaw_delta <= yaw_hi;

  const bool pitch_ok = std::fabs(pitch - gimbal_pitch_) < OUTPOST_PITCH_TOL;

  // 保证底板在前面：
  // 装甲板自身 yaw 和从车到装甲板的观察 yaw 接近，说明这块板正面朝向我方
  const bool bottom_in_front =
      std::fabs(AngleDiff(pre_position_[selected_idx_].yaw, aim_yaw)) < OUTPOST_FRONT_YAW;

  is_fire = bottom_in_front && yaw_ok && pitch_ok;

  last_pitch_ = pitch;
  last_yaw_ = yaw;
  last_x_v_ = target_.velocity.x;
  last_y_v_ = target_.velocity.y;
  last_v_yaw_ = target_.velocity.yaw;
  last_choose_next_ = choose_next_;
  last_outpost_idx_ = target_.outpost_idx;
}

void TrajectorySolver::AutoSolveTrajectory(double& pitch, double& yaw, bool& is_fire,
                                           double& aim_x, double& aim_y, double& aim_z,
                                           int& idx, const Target& target,
                                           double gimbal_yaw, double gimbal_pitch,
                                           const double send_time,
                                           double gimbal_yaw_speed)
{
  target_ = target;
  gimbal_yaw_ = gimbal_yaw;
  gimbal_pitch_ = -gimbal_pitch;
  gimbal_yaw_speed_ = gimbal_yaw_speed;

  fire_logic_mode_ = FireLogicMode::SPIN;

  // 远距离旋转前哨站：只瞄底板，云台停止转动，等底板转到正前方再开火
  // if (IsFarSpinningOutpost())
  // {
  //   SolveFarOutpostBottom(send_time, pitch, yaw, is_fire, aim_x, aim_y, aim_z, idx);
  //   return;
  // }

  // 上游若标记 is_center=false，说明给的就是装甲板的位置/速度
  // 直接走 CV 外推分支，跳过整车建模和择板
  if (!target_.is_center)
  {
    double time_delay = fly_time_ + bias_time_ + send_time;
    SolveByArmorCV(time_delay);
    // 用第一次外推位置估计 fly_time_
    SolvePitch(pre_position_[selected_idx_].x, pre_position_[selected_idx_].y,
               pre_position_[selected_idx_].z);
    time_delay = fly_time_ + bias_time_ + send_time;
    SolveByArmorCV(time_delay);
    UpdateSolveState(pitch, yaw, is_fire, aim_x, aim_y, aim_z, idx);
    return;
  }

  double time_delay = fly_time_ + bias_time_ + send_time;
  AutoSelectArmor(time_delay);
  RCLCPP_ERROR(logger_, " selected_idx_ = %d", selected_idx_);
  PredictOneArmorPosition(time_delay, selected_idx_);
  // 更新fly_time_
  SolvePitch(pre_position_[selected_idx_].x, pre_position_[selected_idx_].y,
             pre_position_[selected_idx_].z);
  time_delay = fly_time_ + bias_time_ + send_time;
  PredictOneArmorPosition(time_delay, selected_idx_);
  UpdateSolveState(pitch, yaw, is_fire, aim_x, aim_y, aim_z, idx);
}

}  // namespace rm_auto_aim
// 没有LOST，预瞄考虑装甲板的位置变化，使用这一时刻与下一时刻的yaw变换计算