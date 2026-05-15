#include "planning_trajectory/trajectory_node.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>

#include "planning_trajectory/trajectory.hpp"
#include "planning_trajectory/trajectory_solver.hpp"

namespace
{
void AddNs(timespec& t, int64_t ns)
{
  t.tv_nsec += ns;
  while (t.tv_nsec >= 1000000000L)
  {
    t.tv_nsec -= 1000000000L;
    ++t.tv_sec;
  }
  while (t.tv_nsec < 0)
  {
    t.tv_nsec += 1000000000L;
    --t.tv_sec;
  }
}

int64_t DiffNs(const timespec& lhs, const timespec& rhs)
{
  return static_cast<int64_t>(lhs.tv_sec - rhs.tv_sec) * 1000000000LL +
         static_cast<int64_t>(lhs.tv_nsec - rhs.tv_nsec);
}
}  // namespace

namespace rm_auto_aim
{
PlanningTrajectoryNode::PlanningTrajectoryNode(const rclcpp::NodeOptions& options)
    : Node("planning_trajectory", options)
{
  this->Init();
  RCLCPP_INFO(this->get_logger(), "Starting PlanningTrajectoryNode!");
}

PlanningTrajectoryNode::~PlanningTrajectoryNode() { StopRtThread(); }

void PlanningTrajectoryNode::TargetCallback(
    const auto_aim_interfaces::msg::Target::SharedPtr target_msg)
{
  TrajectorySolver::Target target_new{};

  target_new.position.x = target_msg->position.x;
  target_new.position.y = target_msg->position.y;
  target_new.position.z = target_msg->position.z;
  target_new.position.yaw = target_msg->yaw;

  target_new.velocity.x = target_msg->velocity.x;
  target_new.velocity.y = target_msg->velocity.y;
  target_new.velocity.z = target_msg->velocity.z;
  target_new.velocity.yaw = target_msg->v_yaw;

  target_new.num = target_msg->armors_num;
  target_new.type = target_msg->type;
  target_new.outpost_idx = target_msg->outpost_idx;

  target_new.radius1 = target_msg->radius_1;
  target_new.radius2 = target_msg->radius_2;

  target_new.number = target_msg->num;
  target_new.is_center = target_msg->is_center;

  {
    std::lock_guard<std::mutex> lk(target_mutex_);

    if (target_msg->is_switchtable && !last_switchtable_)
    {
      switch_table_pending_ = true;
    }
    last_switchtable_ = target_msg->is_switchtable;

    target_ = target_new;
    tracking_ = target_msg->tracking;
    has_target_ = true;

    // A new target snapshot starts a new forward prediction sequence.
    send_time_ = 0.0;

    // Keep Reset() out of the subscription callback. It will be executed
    // in the timer/RT loop, in the same thread that uses trajectory_.
    if (!tracking_)
    {
      planner_reset_pending_ = true;
    }
  }
}

void PlanningTrajectoryNode::PublishStopCommand()
{
  auto_aim_interfaces::msg::Send send_msg;
  send_msg.is_fire = false;
  send_msg.pitch = 0.0;
  send_msg.yaw = 0.0;
  send_msg.vel_yaw = 0.0;
  send_msg.acc_yaw = 0.0;
  send_pub_->publish(send_msg);
}

void PlanningTrajectoryNode::timer_callback() { RtLoopOnce(); }

void PlanningTrajectoryNode::StartRtThread()
{
  if (rt_running_.load())
  {
    return;
  }

  rt_running_.store(true);
  rt_thread_ = std::thread(&PlanningTrajectoryNode::RtLoop, this);
}

void PlanningTrajectoryNode::StopRtThread()
{
  rt_running_.store(false);

  if (rt_thread_.joinable())
  {
    rt_thread_.join();
  }
}

bool PlanningTrajectoryNode::IsCpuAvailable(int cpu) const
{
  if (cpu < 0)
  {
    return false;
  }

  const long online_cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
  return online_cpu_count > 0 && cpu < online_cpu_count;
}

bool PlanningTrajectoryNode::ConfigureCurrentThreadRealtime()
{
  bool ok = true;

  if (rt_enable_cpu_affinity_)
  {
    if (IsCpuAvailable(rt_cpu_))
    {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(rt_cpu_, &cpuset);

      const int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

      if (ret != 0)
      {
        ok = false;
        RCLCPP_WARN(this->get_logger(),
                    "RT loop CPU affinity setup failed, cpu=%d, error=%s. "
                    "Continue without fixed CPU affinity.",
                    rt_cpu_, std::strerror(ret));
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "RT loop bound to CPU%d", rt_cpu_);
      }
    }
    else
    {
      ok = false;
      RCLCPP_WARN(this->get_logger(),
                  "RT loop requested CPU%d, but this machine has fewer online CPUs. "
                  "Continue without fixed CPU affinity.",
                  rt_cpu_);
    }
  }

  if (rt_enable_realtime_)
  {
    const int min_priority = sched_get_priority_min(SCHED_FIFO);
    const int max_priority = sched_get_priority_max(SCHED_FIFO);
    const int priority = std::clamp(rt_priority_, min_priority, max_priority);

    if (priority != rt_priority_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "RT priority %d is out of SCHED_FIFO range [%d, %d], clamped to %d",
                  rt_priority_, min_priority, max_priority, priority);
    }

    sched_param param{};
    param.sched_priority = priority;

    const int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0)
    {
      ok = false;
      RCLCPP_WARN(
          this->get_logger(),
          "RT loop SCHED_FIFO setup failed, priority=%d, error=%s. "
          "Check ulimit -r / realtime group / sudo. Continue with normal scheduling.",
          priority, std::strerror(ret));
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "RT loop uses SCHED_FIFO priority %d", priority);
    }
  }

  return ok;
}

void PlanningTrajectoryNode::RtLoop()
{
#ifdef __linux__
  pthread_setname_np(pthread_self(), "traj_rt_loop");
#endif

  if (rt_lock_memory_)
  {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "mlockall failed: %s. Continue without locked memory.",
                  std::strerror(errno));
    }
  }

  const bool rt_setup_ok = ConfigureCurrentThreadRealtime();

  RCLCPP_INFO(this->get_logger(),
              "Trajectory loop started: mode=%s, period=%.3f ms, rt_setup=%s",
              use_rt_thread_ ? "independent_thread" : "ros_timer", rt_period_ns_ / 1.0e6,
              rt_setup_ok ? "ok" : "degraded");

  timespec next_time{};
  clock_gettime(CLOCK_MONOTONIC, &next_time);

  int64_t max_wake_latency_ns = 0;
  int64_t max_loop_cost_ns = 0;
  int64_t deadline_miss_count = 0;
  int64_t loop_count = 0;

  while (rclcpp::ok() && rt_running_.load())
  {
    AddNs(next_time, rt_period_ns_);

    int sleep_ret = 0;
    do
    {
      sleep_ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
    } while (sleep_ret == EINTR && rt_running_.load());

    if (!rt_running_.load())
    {
      break;
    }

    if (sleep_ret != 0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "clock_nanosleep failed: %s", std::strerror(sleep_ret));
      continue;
    }

    timespec wake_time{};
    clock_gettime(CLOCK_MONOTONIC, &wake_time);
    const int64_t wake_latency_ns = std::max<int64_t>(0, DiffNs(wake_time, next_time));
    max_wake_latency_ns = std::max(max_wake_latency_ns, wake_latency_ns);

    timespec loop_start{};
    clock_gettime(CLOCK_MONOTONIC, &loop_start);

    RtLoopOnce();

    timespec loop_end{};
    clock_gettime(CLOCK_MONOTONIC, &loop_end);
    const int64_t loop_cost_ns = std::max<int64_t>(0, DiffNs(loop_end, loop_start));
    max_loop_cost_ns = std::max(max_loop_cost_ns, loop_cost_ns);

    if (wake_latency_ns + loop_cost_ns > rt_period_ns_)
    {
      ++deadline_miss_count;
    }

    // If this loop is already late by more than one full period, re-align
    // the absolute schedule to avoid a burst of catch-up iterations.
    if (DiffNs(loop_end, next_time) > rt_period_ns_)
    {
      next_time = loop_end;
    }

    ++loop_count;

    if (rt_statistics_interval_ > 0 && loop_count % rt_statistics_interval_ == 0)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "RT loop stats: count=%ld, max_wake=%.3f us, max_cost=%.3f us, miss=%ld",
          loop_count, max_wake_latency_ns / 1000.0, max_loop_cost_ns / 1000.0,
          deadline_miss_count);
    }
  }

  RCLCPP_INFO(
      this->get_logger(),
      "Trajectory loop stopped: count=%ld, max_wake=%.3f us, max_cost=%.3f us, miss=%ld",
      loop_count, max_wake_latency_ns / 1000.0, max_loop_cost_ns / 1000.0,
      deadline_miss_count);
}

void PlanningTrajectoryNode::RtLoopOnce()
{
  bool has_target_local = false;
  bool tracking_local = false;
  bool reset_local = false;
  bool switch_table_local = false;
  double send_time_local = 0.0;
  TrajectorySolver::Target target_local{};

  {
    std::lock_guard<std::mutex> lk(target_mutex_);

    has_target_local = has_target_;
    tracking_local = tracking_;
    target_local = target_;
    reset_local = planner_reset_pending_;
    switch_table_local = switch_table_pending_;

    planner_reset_pending_ = false;
    switch_table_pending_ = false;

    send_time_local = send_time_;
    if (tracking_)
    {
      send_time_ += dt_;
    }
  }

  if (switch_table_local || reset_local)
  {
    std::lock_guard<std::mutex> lk(trajectory_mutex_);
    if (trajectory_)
    {
      if (switch_table_local)
      {
        trajectory_->SwitchTable();
      }

      if (reset_local)
      {
        trajectory_->Reset();
        gimbal_yaw_speed_ = 0.0;
      }
    }
  }

  if (!has_target_local || !tracking_local)
  {
    PublishStopCommand();
    return;
  }

  double gimbal_yaw = 0.0;
  double gimbal_pitch = 0.0;

  try
  {
    const auto gimbal_yaw_pitch = GetGimbalYawAndPitch();
    gimbal_yaw = gimbal_yaw_pitch.first;
    gimbal_pitch = gimbal_yaw_pitch.second;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Get gimbal transform failed: %s", ex.what());
    PublishStopCommand();
    return;
  }

  double aim_x = 0.0;
  double aim_y = 0.0;
  double aim_z = 0.0;
  int idx = TrajectorySolver::LOST;
  TrajectorySolver::control cmd{};

  double bc_yaw = 0.0;
  double bc_pitch = 0.0;

  {
    std::lock_guard<std::mutex> lk(trajectory_mutex_);

    if (!trajectory_)
    {
      PublishStopCommand();
      return;
    }

    trajectory_->solver().AutoSolveTrajectory(
        cmd.pitch, cmd.yaw, cmd.is_fire, aim_x, aim_y, aim_z, idx, target_local,
        gimbal_yaw, gimbal_pitch, send_time_local, gimbal_yaw_speed_);

    bc_yaw = cmd.yaw;
    bc_pitch = cmd.pitch;

    // Keep the current behavior of the uploaded code: UpdatePlanTrajectory()
    // remains disabled. If you want the constrained planner to update each
    // cycle, uncomment the next line after verifying planner timing.
    // trajectory_->UpdatePlanTrajectory(cmd, gimbal_yaw);

    gimbal_yaw_speed_ = cmd.vel_yaw;
  }

  auto_aim_interfaces::msg::Send send_msg;
  send_msg.is_fire = cmd.is_fire;

  if (std::fabs(gimbal_yaw - bc_yaw) > M_PI / 2)
  {
    send_msg.pitch = 0;
    send_msg.yaw = 0;
  }
  else
  {
    send_msg.pitch = bc_pitch;
    send_msg.yaw = bc_yaw;
  }

  // Keep the current uploaded behavior: yaw velocity/acceleration output is
  // forced to zero. Replace with cmd.vel_yaw/cmd.acc_yaw if the controller
  // needs feed-forward.
  send_msg.vel_yaw = 0.0;
  send_msg.acc_yaw = 0.0;
  send_msg.num = target_local.number;
  send_pub_->publish(send_msg);

  auto_aim_interfaces::msg::TrajectoryInfo info_msg;
  info_msg.aim_position.x = aim_x;
  info_msg.aim_position.y = aim_y;
  info_msg.aim_position.z = aim_z;
  info_msg.gimbal_yaw = gimbal_yaw;
  info_msg.gimbal_pitch = -gimbal_pitch;
  info_msg.idx = idx;
  info_msg.bc_yaw = bc_yaw;
  info_msg.bc_pitch = bc_pitch;
  info_pub_->publish(info_msg);
}

std::pair<double, double> PlanningTrajectoryNode::GetGimbalYawAndPitch()
{
  std::pair<double, double> gimbal_yaw_pitch{0.0, 0.0};

  const auto transform_stamped_yaw =
      tf2_buffer_->lookupTransform("gimbal_odom", "yaw_link", tf2::TimePointZero);
  const auto transform_stamped_pitch =
      tf2_buffer_->lookupTransform("gimbal_odom", "pitch_link", tf2::TimePointZero);

  tf2::Quaternion q_yaw(transform_stamped_yaw.transform.rotation.x,
                        transform_stamped_yaw.transform.rotation.y,
                        transform_stamped_yaw.transform.rotation.z,
                        transform_stamped_yaw.transform.rotation.w);
  tf2::Quaternion q_pitch(transform_stamped_pitch.transform.rotation.x,
                          transform_stamped_pitch.transform.rotation.y,
                          transform_stamped_pitch.transform.rotation.z,
                          transform_stamped_pitch.transform.rotation.w);

  double roll = 0.0, pitch = 0.0, yaw = 0.0;

  tf2::Matrix3x3(q_yaw).getRPY(roll, pitch, yaw);
  gimbal_yaw_pitch.first = yaw;

  tf2::Matrix3x3(q_pitch).getRPY(roll, pitch, yaw);
  gimbal_yaw_pitch.second = pitch;

  return gimbal_yaw_pitch;
}

void PlanningTrajectoryNode::Init()
{
  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // SolveTarget init parameters
  double k = this->declare_parameter("k", 0.092);
  double bias_time = this->declare_parameter("bias_time", 0.01);
  double s_bias = this->declare_parameter("s_bias", 0.0);
  double z_bias = this->declare_parameter("z_bias", 0.0);
  double pitch_bias = this->declare_parameter("pitch_bias", 0.0);
  send_frequency_ = this->declare_parameter("send_frequency", 200.0);
  if (send_frequency_ <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid send_frequency %.3f, fallback to 200 Hz",
                send_frequency_);
    send_frequency_ = 200.0;
  }
  dt_ = 1.0 / send_frequency_;
  rt_period_ns_ = std::max<int64_t>(1, static_cast<int64_t>(1.0e9 / send_frequency_));

  use_rt_thread_ = this->declare_parameter("rt.use_rt_thread", true);
  rt_cpu_ = this->declare_parameter("rt.cpu", 7);
  rt_priority_ = this->declare_parameter("rt.priority", 80);
  rt_enable_cpu_affinity_ = this->declare_parameter("rt.enable_cpu_affinity", true);
  rt_enable_realtime_ = this->declare_parameter("rt.enable_realtime", true);
  rt_lock_memory_ = this->declare_parameter("rt.lock_memory", true);
  rt_statistics_interval_ =
      this->declare_parameter("rt.statistics_interval", static_cast<int64_t>(0));

  bool use_table = this->declare_parameter("calculate_mode", true);

  double max_x = this->declare_parameter("table.max_x", 13.0);
  double min_x = this->declare_parameter("table.min_x", 0.0);
  double max_y = this->declare_parameter("table.max_y", 2.0);
  double min_y = this->declare_parameter("table.min_y", -1.0);
  double resolution = this->declare_parameter("table.resolution", 0.01);

  double max_x_lob = this->declare_parameter("table.max_x_lob", 22.0);
  double min_x_lob = this->declare_parameter("table.min_x_lob", 0.0);
  double max_y_lob = this->declare_parameter("table.max_y_lob", 3.0);
  double min_y_lob = this->declare_parameter("table.min_y_lob", -1.0);
  double resolution_lob = this->declare_parameter("table.resolution_lob", 0.01);

  k_yaw_ = this->declare_parameter("k_yaw", 0.0);
  k_pitch_ = this->declare_parameter("k_pitch", 0.0);

  std::string package_prefix =
      ament_index_cpp::get_package_share_directory("rm_vision_bringup") + "/config/";
  table_filename_normal_ =
      package_prefix + this->declare_parameter("table.filename", "table.bin");
  RCLCPP_ERROR(this->get_logger(), "table_filename_normal_: %s",
               table_filename_normal_.c_str());
  auto robot_type = this->declare_parameter<std::string>("robot_type", "default");
  is_hero_ = (robot_type == "hero");

  if (is_hero_)
  {
    table_filename_lob_ =
        package_prefix + this->declare_parameter("table.filename_lob", "");
    RCLCPP_ERROR(this->get_logger(), "table_filename_lob_: %s",
                 table_filename_lob_.c_str());
  }

  TrajectorySolver::CalculateMode calculate_mode =
      use_table ? TrajectorySolver::CalculateMode::TABLE_LOOKUP
                : TrajectorySolver::CalculateMode::NORMAL;

  table_config_ = {max_x, min_x, max_y, min_y, resolution, table_filename_normal_};
  if (is_hero_)
  {
    table_config_lob_ = {max_x_lob, min_x_lob,      max_y_lob,
                         min_y_lob, resolution_lob, table_filename_lob_};
  }
  else
  {
    table_config_lob_ = table_config_;
  }

  timer_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_cb_group_;

  velocity_sub_ = this->create_subscription<auto_aim_interfaces::msg::Velocity>(
      "/current_velocity",
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
      [this](const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
      {
        std::lock_guard<std::mutex> lk(trajectory_mutex_);
        if (trajectory_)
        {
          trajectory_->InitVelocity(velocity_msg);
        }
      },
      sub_options);

  auto target_qos = rclcpp::QoS(1);
  target_qos.best_effort();
  target_qos.durability_volatile();

  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", target_qos,
      [this](const auto_aim_interfaces::msg::Target::SharedPtr target_msg)
      { TargetCallback(target_msg); },
      sub_options);

  send_pub_ = this->create_publisher<auto_aim_interfaces::msg::Send>(
      "/trajectory/send", rclcpp::SensorDataQoS());
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrajectoryInfo>(
      "/trajectory/info", rclcpp::SensorDataQoS());

  q_yaw_ = this->declare_parameter("ekf.q_yaw", 0.0);
  q_pitch_ = this->declare_parameter("ekf.q_pitch", 0.0);
  q_jerk_ = this->declare_parameter("ekf.q_jerk", 0.0);
  r_yaw_ = this->declare_parameter("ekf.r_yaw", 0.0);
  r_pitch_ = this->declare_parameter("ekf.r_pitch", 0.0);
  auto f = [this](const Eigen::VectorXd& x) -> Eigen::VectorXd
  {
    Eigen::VectorXd x_new(4);
    x_new(0) = x(0) + x(1) * dt_ + 0.5 * x(2) * dt_ * dt_;
    x_new(1) = x(1) + x(2) * dt_;
    x_new(2) = x(2);
    x_new(3) = x(3);
    return x_new;
  };

  auto h = [](const Eigen::VectorXd& x) -> Eigen::VectorXd
  {
    Eigen::VectorXd z(2);
    z(0) = x(0);  // yaw
    z(1) = x(3);  // pitch
    return z;
  };

  // ---- 过程函数雅可比 j_f ----
  auto j_f = [this](const Eigen::VectorXd&) -> Eigen::MatrixXd
  {
    Eigen::MatrixXd F(4, 4);
    // clang-format off
    F << 1, dt_, 0.5 * dt_ * dt_, 0,
         0, 1,   dt_,             0,
         0, 0,   1,               0,
         0, 0,   0,               1;
    // clang-format on
    return F;
  };

  auto j_h = [](const Eigen::VectorXd&) -> Eigen::MatrixXd
  {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
    H(0, 0) = 1.0;  // yaw
    H(1, 3) = 1.0;  // pitch
    return H;
  };
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);

  // yaw: 独立噪声
  Q(0, 0) = q_yaw_;
  double T = dt_;  // 采样周期
  double T2 = T * T;
  double T3 = T2 * T;

  Q(1, 1) = q_jerk_ * T3 / 3.0;  // vy-vy
  Q(1, 2) = q_jerk_ * T2 / 2.0;  // vy-ay
  Q(2, 1) = q_jerk_ * T2 / 2.0;  // ay-vy
  Q(2, 2) = q_jerk_ * T;         // ay-ay

  // pitch: 独立噪声
  Q(3, 3) = q_pitch_;

  auto u_q = [Q]() -> Eigen::MatrixXd { return Q; };

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
  R(0, 0) = r_yaw_;
  R(1, 1) = r_pitch_;
  auto u_r = [R](const Eigen::VectorXd&) -> Eigen::MatrixXd { return R; };

  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4);
  P0(0, 0) = 0;
  P0(1, 1) = 1000.0;
  P0(2, 2) = 1000.0;
  P0(3, 3) = 0;

  TrajectorySolver solver(k, bias_time, s_bias, z_bias, pitch_bias, calculate_mode,
                          table_config_, table_config_lob_);
  ExtendedKalmanFilter ekf(f, h, j_f, j_h, u_q, u_r, P0);
  ConstrainedPlanner planner(25, 20, 500, dt_);

  trajectory_ = std::make_unique<Trajectory>(solver, std::move(ekf), std::move(planner));

  if (use_rt_thread_)
  {
    StartRtThread();
  }
  else
  {
    const auto timer_period =
        std::chrono::nanoseconds(static_cast<int64_t>(rt_period_ns_));

    timer_ = this->create_wall_timer(
        timer_period, std::bind(&PlanningTrajectoryNode::timer_callback, this),
        timer_cb_group_);

    RCLCPP_INFO(this->get_logger(),
                "Trajectory loop started: mode=ros_timer, period=%.3f ms",
                rt_period_ns_ / 1.0e6);
  }
}
}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its
// library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::PlanningTrajectoryNode)
