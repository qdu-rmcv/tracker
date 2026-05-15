#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <algorithm>
#include <cmath>
#include <numbers>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "planning_trajectory/trajectory_solver.hpp"

namespace rm_auto_aim
{
class ConstrainedPlanner
{
 public:
  ConstrainedPlanner(double wn, double v_max, double a_max, double dt)
      : wn_(wn), v_max_(v_max), a_max_(a_max), dt_(dt) {}

  void setState(double theta0, double v0 = 0.0);
  void update(double target_pos, double target_vel = 0.0, double target_acc = 0.0);

  double getTheta() const { return theta_cmd_; }
  double getV() const { return v_cmd_; }
  double getA_cmd() const { return a_cmd_; }

 private:
  double wn_;     // rad/s
  double v_max_;  // rad/s
  double a_max_;  // rad/s^2
  double dt_;     // s

  double theta_cmd_{0.0};
  double v_cmd_{0.0};
  double a_cmd_{0.0};
};

class Trajectory
{
 public:
  Trajectory(TrajectorySolver solver, ExtendedKalmanFilter ekf,
             ConstrainedPlanner planner);
  ~Trajectory() = default;

  void Reset();
  void InitVelocity(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
  {
    solver_.Init(velocity_msg);
  }
  bool SwitchTable() { return solver_.SwitchTable(); }

  TrajectorySolver& solver() { return solver_; }
  const TrajectorySolver& solver() const { return solver_; }

  void UpdatePlanTrajectory(TrajectorySolver::control& cmd,
                            const double gimbal_yaw);

 private:
  static double NormalizeAngle(double a)
  {
    return std::remainder(a, 2.0 * std::numbers::pi_v<double>);
  }

  static double UnwrapAngle(double wrapped, double last_unwrapped)
  {
    return last_unwrapped + NormalizeAngle(wrapped - NormalizeAngle(last_unwrapped));
  }

  TrajectorySolver solver_;
  ExtendedKalmanFilter ekf_;
  ConstrainedPlanner planner_;

  bool ekf_initialized_{false};
  bool planner_initialized_{false};
  double last_unwrapped_yaw_{0.0};
};
}  // namespace rm_auto_aim

#endif
