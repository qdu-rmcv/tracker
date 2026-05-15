#include "planning_trajectory/trajectory.hpp"

#include <cmath>

namespace rm_auto_aim
{
void ConstrainedPlanner::setState(double theta0, double v0)
{
  theta_cmd_ = theta0;
  v_cmd_ = v0;
  a_cmd_ = 0.0;
}

void ConstrainedPlanner::update(double target_pos, double target_vel, double target_acc)
{
  double e = target_pos - theta_cmd_;
  double ve = v_cmd_ - target_vel;

  double d_brake = (ve * ve) / (2.0 * a_max_);
  bool approaching = (e > 0.0 && v_cmd_ > target_vel) || (e < 0.0 && v_cmd_ < target_vel);
  bool too_close = std::abs(e) <= d_brake;

  double vel_thresh = std::abs(target_vel) * 0.1 + 1.0;
  bool wrong_direction =
      (e >= 0.0 && v_cmd_ < -vel_thresh) || (e < 0.0 && v_cmd_ > vel_thresh);

  double a_desired = 0.0;

  if (approaching && too_close)
  {
    if (std::abs(e) > 1e-6)
    {
      a_desired = (target_vel * target_vel - v_cmd_ * v_cmd_) / (2.0 * e);
    }
    else
    {
      a_desired = -std::copysign(a_max_, ve);
    }
  }
  else if (wrong_direction)
  {
    a_desired = -std::copysign(a_max_, v_cmd_);
  }
  else
  {
    a_desired = wn_ * wn_ * e - 2.0 * wn_ * ve + target_acc;
  }

  a_desired = std::clamp(a_desired, -a_max_, a_max_);

  const double v_old = v_cmd_;
  v_cmd_ += a_desired * dt_;
  v_cmd_ = std::clamp(v_cmd_, -v_max_, v_max_);
  a_cmd_ = (v_cmd_ - v_old) / dt_;
  theta_cmd_ += v_cmd_ * dt_;
}

Trajectory::Trajectory(TrajectorySolver solver, ExtendedKalmanFilter ekf,
                       ConstrainedPlanner planner)
    : solver_(std::move(solver)), ekf_(std::move(ekf)), planner_(std::move(planner))
{
}

void Trajectory::Reset()
{
  ekf_initialized_ = false;
  planner_initialized_ = false;
  last_unwrapped_yaw_ = 0.0;
  solver_.ReBuild();
}

void Trajectory::UpdatePlanTrajectory(TrajectorySolver::control& cmd,
                                      const double gimbal_yaw)
{
  const double meas_yaw =
      ekf_initialized_ ? UnwrapAngle(cmd.yaw, last_unwrapped_yaw_) : cmd.yaw;
  last_unwrapped_yaw_ = meas_yaw;

  if (!ekf_initialized_)
  {
    Eigen::VectorXd x0(4);
    x0 << meas_yaw, 0.0, 0.0, cmd.pitch;
    ekf_.SetState(x0);
    ekf_initialized_ = true;
  }

  Eigen::VectorXd z(2);
  z << meas_yaw, cmd.pitch;

  ekf_.Predict();
  Eigen::VectorXd x = ekf_.Update(z);

  if (!planner_initialized_)
  {
    const double gimbal_yaw_unwrapped = UnwrapAngle(gimbal_yaw, x(0));
    planner_.setState(gimbal_yaw_unwrapped, 0.0);
    planner_initialized_ = true;
  }

  planner_.update(x(0), x(1), x(2));

  cmd.yaw = NormalizeAngle(planner_.getTheta());
  cmd.vel_yaw = planner_.getV();
  cmd.acc_yaw = planner_.getA_cmd();
  cmd.pitch = x(3);
}
}  // namespace rm_auto_aim
