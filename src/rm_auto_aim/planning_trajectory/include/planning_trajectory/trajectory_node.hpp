#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <thread>
#include <utility>

#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/trajectory_info.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"
#include "planning_trajectory/trajectory.hpp"
#include "planning_trajectory/trajectory_solver.hpp"

namespace rm_auto_aim
{
class PlanningTrajectoryNode : public rclcpp::Node
{
 public:
  explicit PlanningTrajectoryNode(const rclcpp::NodeOptions& options);
  ~PlanningTrajectoryNode() override;

 private:
  void Init();
  void TargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg);

  // timer_callback is kept as a fallback path when rt.use_rt_thread is false.
  void timer_callback();

  void StartRtThread();
  void StopRtThread();
  void RtLoop();
  void RtLoopOnce();
  bool ConfigureCurrentThreadRealtime();
  bool IsCpuAvailable(int cpu) const;

  std::pair<double, double> GetGimbalYawAndPitch();
  void PublishStopCommand();

  std::unique_ptr<Trajectory> trajectory_;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  // Protects target snapshot and lightweight state copied from subscriptions.
  std::mutex target_mutex_;

  // Protects trajectory_, solver internal state, SwitchTable(), Reset(),
  // InitVelocity(), and gimbal_yaw_speed_.
  std::mutex trajectory_mutex_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::TrajectoryInfo>::SharedPtr info_pub_;

  std::string table_filename_normal_;
  std::string table_filename_lob_;
  Table::TableConfig table_config_;
  Table::TableConfig table_config_lob_;

  double k_yaw_{0.0};
  double k_pitch_{0.0};
  bool is_hero_{false};

  bool has_target_{false};
  bool tracking_{false};
  bool last_switchtable_{false};
  bool switch_table_pending_{false};
  bool planner_reset_pending_{true};

  TrajectorySolver::Target target_{};
  double send_frequency_{200.0};
  double dt_{0.005};
  double send_time_{0.0};
  double gimbal_yaw_speed_{0.0};

  // Independent real-time loop parameters.
  bool use_rt_thread_{true};
  bool rt_enable_cpu_affinity_{true};
  bool rt_enable_realtime_{true};
  bool rt_lock_memory_{true};
  int rt_cpu_{7};
  int rt_priority_{80};
  int64_t rt_period_ns_{5000000};
  int64_t rt_statistics_interval_{0};

  std::thread rt_thread_;
  std::atomic_bool rt_running_{false};

  double q_yaw_{0.0};
  double q_pitch_{0.0};
  double q_jerk_{0.0};
  double r_yaw_{0.0};
  double r_pitch_{0.0};
};
}  // namespace rm_auto_aim

#endif
