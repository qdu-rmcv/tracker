#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// ROS
#include <angles/angles.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "armor_tracker/tracker_params.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{

// 装甲板数量：正常 4 块 / 前哨站 3 块
enum class ArmorsNum : uint8_t
{
  NORMAL_4 = 4,
  OUTPOST_3 = 3
};

class Tracker  // 整车观测
{
 public:
  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  enum class State : uint8_t
  {
    LOST = 0,
    DETECTING = 1,
    TRACKING = 2,
    TEMP_LOST = 3,
  };

  // 当前对外输出选用的模型
  enum class ModelType : uint8_t
  {
    FULL = 0,    // 整车模型 (8 维)
    ARMOR = 1,   // 装甲板 CV 模型 (5 维)
    OUTPOST = 2  // 前哨站模型 (5 维)
  };

  // 对外输出，Node 端按字段填充 target_msg
  struct Output
  {
    bool is_center = true;           // true = 整车/前哨 (xc,yc), false = 装甲板 (xa,ya)
    Eigen::Vector3d position{0, 0, 0};
    Eigen::Vector3d velocity{0, 0, 0};
    double yaw = 0.0;
    double v_yaw = 0.0;
    double radius_1 = 0.0;
    double radius_2 = 0.0;
    double dz = 0.0;
    int armors_num = 4;
    int outpost_idx = 0;
    std::string armor_type;
    std::string armor_number;
  };

  explicit Tracker(const TrackerParams& params);

  // ---------- Node 每帧调用 ----------
  void SetCameraPose(const Eigen::Matrix3d& rot_camera_to_world,
                     const Eigen::Vector3d& camera_origin_world);
  void SetDt(double dt) { dt_ = dt; }

  // ---------- 核心流程 ----------
  void Init(const Armors::SharedPtr& armors_msg);
  void Update(const Armors::SharedPtr& armors_msg);

  // ---------- 状态机参数（由 node 根据 dt 设置） ----------
  void SetTimingThres(int lost_thres, int change_thres)
  {
    lost_thres_ = lost_thres;
    change_thres_ = change_thres;
  }

  // ---------- 输出 ----------
  Output GetOutput() const { return output_; }
  State GetTrackerState() const { return tracker_state_; }
  bool IsOutpostMode() const { return is_outpost_; }

  // ---------- 调试信息（Node 发布 TrackerInfo 用） ----------
  Eigen::VectorXd measurement = Eigen::VectorXd::Zero(4);  // [xa, ya, za, yaw]
  double info_position_diff = 0.0;
  double info_yaw_diff = 0.0;

 private:
  // -------------------------------------------------------------------
  //  EKF 构建（实例化时注入 lambda）
  // -------------------------------------------------------------------
  void BuildEkfFull();
  void BuildEkfArmor();
  void BuildEkfOutpost();

  // -------------------------------------------------------------------
  //  普通模式 (非前哨) 流程
  // -------------------------------------------------------------------
  void UpdateNormal(const Armors::SharedPtr& armors_msg);
  void InitNormal(const Armor& armor);

  // 用整车模型预测做几何匹配（多装甲板 + yaw 跳变检测）
  bool MatchArmorFull(const std::vector<Armor>& target_id_armors,
                      const Eigen::VectorXd& full_pred, double& position_diff,
                      double& yaw_diff, bool& is_jump);

  void HandleArmorJumpFull(const Armor& current_armor);
  void HandleArmorJumpArmor(const Armor& current_armor);  // 重置位置保留速度+协方差

  void UpdateEkfFull(double measured_yaw, const geometry_msgs::msg::Point& armor_pos);
  void UpdateEkfArmor(const geometry_msgs::msg::Point& armor_pos);

  void ClampFullRadius();
  void DoYouWantToChangeTarget(const Armors::SharedPtr& armors_msg);
  void ResetStateFull(double yaw, const geometry_msgs::msg::Point& p);

  // 选择对外模型（带迟滞）
  void SelectActiveModel();

  // -------------------------------------------------------------------
  //  前哨站模式流程
  // -------------------------------------------------------------------
  void UpdateOutpost(const Armors::SharedPtr& armors_msg);
  void InitOutpost(const Armor& armor);

  bool MatchArmorOutpost(const std::vector<Armor>& target_id_armors,
                         const Eigen::VectorXd& outpost_pred, double& position_diff,
                         double& yaw_diff, bool& is_jump);

  void HandleArmorJumpOutpost(const Armor& current_armor);
  void UpdateEkfOutpost(double measured_yaw,
                        const geometry_msgs::msg::Point& armor_pos);

  // 更新当前观测装甲板 idx：跳变法 + 几何法
  void UpdateOutpostIdx(const geometry_msgs::msg::Point& armor_pos, bool is_jump);

  // 旋转/静止判定 + v_yaw 钳制（学习期后启用）
  void ApplyOutpostMotionLogic();

  // -------------------------------------------------------------------
  //  公共工具
  // -------------------------------------------------------------------
  void UpdateTrackerState(bool matched);
  void UpdateArmorsNum(const Armor& armor);

  double OrientationToYaw(const geometry_msgs::msg::Quaternion& q);
  Eigen::Vector3d GetArmorPositionFromFullState(const Eigen::VectorXd& x) const;
  Eigen::Vector3d GetArmorPositionFromOutpostState(const Eigen::VectorXd& x) const;

  static double NormalizeAngle(double a) { return std::remainder(a, 2.0 * M_PI); }
  static double AngleDiff(double a, double b) { return NormalizeAngle(a - b); }

  // 把对应模型的状态打包成 Output
  void FillOutputFromFull(bool is_center);
  void FillOutputFromOutpost();

  Eigen::Matrix3d BuildJacobianYpdToCameraXyz(const Eigen::Vector3d& p_cam) const;

  // -------------------------------------------------------------------
  //  成员
  // -------------------------------------------------------------------
  TrackerParams params_;

  // 三个 EKF
  ExtendedKalmanFilter ekf_full_;
  ExtendedKalmanFilter ekf_armor_;
  ExtendedKalmanFilter ekf_outpost_;

  // 当前选用的对外模型（普通模式下在 FULL/ARMOR 之间切换；前哨模式恒为 OUTPOST）
  ModelType active_model_ = ModelType::FULL;

  // 是否处于前哨模式
  bool is_outpost_ = false;

  // 状态机
  State tracker_state_ = State::LOST;

  // 跟踪信息
  std::string tracked_id_;
  Armor tracked_armor_;
  Armor last_tracked_armor_{};
  ArmorsNum tracked_armors_num_ = ArmorsNum::NORMAL_4;

  // 输出
  Output output_;

  // 时间步长（由 node 设置）
  double dt_ = 0.01;

  // 相机位姿（u_r 使用）
  Eigen::Matrix3d camera_to_world_rot_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d camera_origin_world_ = Eigen::Vector3d::Zero();

  // 整车模型当前预测（供 MatchArmor / 输出对齐使用）
  Eigen::VectorXd full_pred_;
  Eigen::Vector3d predicted_armor_pos_full_{0, 0, 0};

  // 装甲板（公共目标信息）
  double another_r_ = 0.26;
  double dz_ = 0.0;

  // 前哨站：当前被观测装甲板索引（0=高 / 1=中 / 2=低）
  int outpost_idx_ = 0;

  // 状态机计数
  int detect_count_ = 0;
  int lost_count_ = 0;
  int change_count_ = 0;
  int lost_thres_ = 30;
  int change_thres_ = 30;
  int jump_cooldown_ = 0;
  static constexpr int JUMP_COOLDOWN_FRAMES = 3;

  std::string last_closest_id_;
  double last_yaw_unwrap_ = 0.0;

  // 整车模型 r 锁定相关（高速旋转时）
  int full_update_count_ = 0;
  double full_last_r_ = 0.26;

  // 前哨站学习/运动状态
  enum class OutpostMotion : uint8_t
  {
    LEARNING = 0,  // 学习期内
    STATIC = 1,    // 静止
    ROTATING = 2,  // 旋转
  };
  OutpostMotion outpost_motion_ = OutpostMotion::LEARNING;
  int outpost_update_count_ = 0;
  double outpost_v_yaw_avg_ = 0.0;  // 学习期内 v_yaw 累积平均
  double outpost_z_avg_ = 0.0;      // 学习期内 z_c 累积平均
  int outpost_v_yaw_sign_ = 1;      // 锁定后的 v_yaw 方向 (±1)
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
