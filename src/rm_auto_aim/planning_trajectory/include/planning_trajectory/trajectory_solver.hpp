#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"
#include "planning_trajectory/table.hpp"

namespace rm_auto_aim
{
using time_point = std::chrono::steady_clock::time_point;

class TrajectorySolver
{
 public:
  static constexpr double GRAVITY = 9.78;
  static constexpr double SMALL_HALF_LENGTH = 135.0 / 2.0 / 1000.0;
  static constexpr double LARGE_HALF_LENGTH = 230.0 / 2.0 / 1000.0;
  static constexpr double outpost_dz = 0.1;

  enum CalculateMode : uint8_t
  {
    NORMAL = 0,
    TABLE_LOOKUP = 1
  };

  enum class FireLogicMode
  {
    OUTPOST,
    SPIN,
    SPIN_TEMP,
    COMMON,
    BUFF
  };

  enum SpecialArmor : int8_t
  {
    LOST = -2,
    CENTER = -1
  };

  struct TarPostion
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
  };

  struct TarVelocity
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
  };

  struct Target
  {
    TarPostion position{};
    TarVelocity velocity{};
    int num{0};
    std::string type{};
    double radius1{0.0};
    double radius2{0.0};
    int outpost_idx{0};
    int number{0};
    // 上游 tracker 输出的是车辆中心 (true) 还是装甲板本身 (false)
    bool is_center{true};
  };

  struct control
  {
    double yaw{0.0};
    double pitch{0.0};
    double vel_yaw{0.0};
    double acc_yaw{0.0};
    bool is_fire{false};
  };

  TrajectorySolver(const double& k, const double& bias_time, const double& s_bias,
                   const double& z_bias, const double& pitch_bias,
                   CalculateMode calculate_mode, const Table::TableConfig& table_config,
                   const Table::TableConfig& table_config_lob);

  void Init(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg);
  void ReBuild();

  double MonoDirectionalAirResistanceModel(double s, double angle, double v);
  TarPostion PredictCenter(double time_delay);
  TarPostion PredictArmor(int idx, const TarPostion& pre_center);
  void PredictAllArmorPosition(double time_delay);
  void PredictOneArmorPosition(double time_delay, int idx);

  double SolvePitch(double x, double y, double z);
  double SolveYaw(double x, double y) const;
  // 返回以 (target.x, target.y) 为原点的允许开火 yaw 窗口 [min, max]
  // delta = tar_yaw - gimbal_yaw 落在此区间即可命中
  std::pair<double, double> ComputeFireYawWindow(const TarPostion& armor) const;
  double GetArmorHalfWidth() const;

  bool CanFire(double yaw, double pitch, bool is_fast_fire = false);
  void GlobalSelectArmor(double time_delay);
  void LocalSelectArmor(double time_delay);
  void LocalSelectArmorForHero(double time_delay);
  void PreSelectArmor(double time_delay);
  void AutoSelectArmor(double time_delay, bool is_pre_select = false);

  bool IsFarSpinningOutpost() const;
  int SelectOutpostBottomArmor() const;
  void SolveFarOutpostBottom(double send_time, double& pitch, double& yaw, bool& is_fire,
                             double& aim_x, double& aim_y, double& aim_z, int& idx);

  // 装甲板 CV 模型分支：上游已经给装甲板位置和速度，不再建模、不再择板
  void SolveByArmorCV(double time_delay);

  void UpdateFireLogicMode();
  void UpdateSolveState(double& pitch, double& yaw, bool& is_fire, double& aim_x,
                        double& aim_y, double& aim_z, int& idx);

  void AutoSolveTrajectory(double& pitch, double& yaw, bool& is_fire, double& aim_x,
                           double& aim_y, double& aim_z, int& idx, const Target& target,
                           double gimbal_yaw, double gimbal_pitch, const double send_time,
                           double gimbal_yaw_speed);

  void SetTarget(const Target& t) { target_ = t; }

  bool SwitchTable();

 private:
  static double NormalizeAngle(double a)
  {
    return std::remainder(a, 2.0 * std::numbers::pi_v<double>);
  }

  static double AngleDiff(double a, double b) { return NormalizeAngle(a - b); }

  bool HasValidSelection() const
  {
    return target_.num > 0 && selected_idx_ >= 0 && selected_idx_ < target_.num &&
           selected_idx_ < static_cast<int>(pre_position_.size());
  }

  void ResetFireState();

  rclcpp::Logger logger_{rclcpp::get_logger("planning_trajectory")};

  double current_v_{12.0};
  double fly_time_{0.0};

  time_point start_turn_{time_point::min()};
  time_point end_turn_{time_point::min()};
  time_point last_start_turn_{time_point::min()};

  double turn_s_{0.0};
  double step_s_{0.0};
  double selected_time_delay_{0.0};

  TarPostion pre_center_{};
  std::array<TarPostion, 4> pre_position_{};
  Target target_{};

  static constexpr double FAR_OUTPOST_DISTANCE = 6.0;
  static constexpr double OUTPOST_SPIN_VYAW = 1.0;
  static constexpr double OUTPOST_FRONT_YAW = 0.35;
  static constexpr double OUTPOST_PITCH_TOL = 0.025;

  double gimbal_yaw_{0.0};
  double gimbal_pitch_{0.0};

  std::shared_ptr<Table> table_;
  std::shared_ptr<Table> table_lob_;
  std::shared_ptr<Table> current_table_;
  CalculateMode calculate_mode_{CalculateMode::TABLE_LOOKUP};
  FireLogicMode fire_logic_mode_{FireLogicMode::COMMON};

  double k_{0.0};
  double pitch_bias_{0.0};
  double bias_time_{0.0};
  double s_bias_{0.0};
  double z_bias_{0.0};

  double last_pitch_{0.0};
  double last_yaw_{0.0};
  int selected_idx_{SpecialArmor::LOST};
  int last_selected_idx_for_turn_{LOST};

  double last_x_v_{0.0};
  double last_y_v_{0.0};
  double last_v_yaw_{0.0};
  bool last_choose_next_{false};

  bool choose_next_{false};
  bool should_last_shot_{false};
  double gimbal_yaw_speed_{0.0};

  bool no_fire_{false};
  int choose_next_count_{0};

  int last_outpost_idx_{-1};
};

}  // namespace rm_auto_aim
