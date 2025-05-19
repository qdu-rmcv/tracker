// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/tracker_node.hpp"

// STD
#include <auto_aim_interfaces/msg/detail/all_latency__struct.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // 用于弹道解算的k值具体计算在git
  float k = this->declare_parameter("tracker.k", 0.092);
  int bias_time = this->declare_parameter("tracker.bias_time", 100);
  float s_bias = this->declare_parameter("tracker.s_bias", 0.18375);
  float z_bias = this->declare_parameter("tracker.z_bias", 0.196);

  // 判断开火范围
  float fire_k = this->declare_parameter("tracker.fire_k", 0.001);

  // // v_yaw 切换,判断攻击装甲板还是攻击车中心
  // float use_v_yaw = this->declare_parameter("use_v_yaw", 15.0);

  // 解算的智能指针
  gaf_solver = std::make_unique<SolveTrajectory>(k, bias_time, s_bias, z_bias, fire_k);



  //! EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  // auto f = [this](const Eigen::VectorXd & x) {
  //   Eigen::VectorXd x_new = x;
  //   // x_new(0) += x(1) * dt_;
  //   // x_new(2) += x(3) * dt_;
  //   // x_new(4) += x(5) * dt_;
  //   // x_new(6) += x(7) * dt_;

  //   x_new(0) += x(1) * dt_;
  //   x_new(2) += x(3) * dt_;
  //   x_new(4) += x(5) * dt_;
  //   // yaw 的状态转移
  //   x_new(6) += x(7) * dt_ + 0.5 * x(8) * dt_ * dt_ + (1.0 / 6.0) * x(9) * dt_ * dt_ * dt_;
  //   x_new(7) += x(8) * dt_ + 0.5 * x(9) * dt_ * dt_;
  //   x_new(8) += x(9) * dt_;
  //   x_new(9) = x(9);  // 加加速度恒定
  //   // r 的状态转移
  //   x_new(10) = x(10);

  //   return x_new;
  // };

  auto f_a = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    x_new(4) += x(5) * dt_;
    // yaw 的状态转移
    x_new(6) += x(7) * dt_ + 0.5 * x(8) * dt_ * dt_ ;
    x_new(7) += x(8) * dt_;
    x_new(8) = x(8);  // 加速度恒定 
    // r 的状态转移
    x_new(9) = x(9);

    return x_new;
  };

  // J_f - Jacobian of process function
  // auto j_f = [this](const Eigen::VectorXd &) {
  //   // Eigen::MatrixXd f(9, 9);
  //   // // clang-format off
  //   // f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
  //   //       0,   1,   0,   0,   0,   0,   0,   0,   0,
  //   //       0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
  //   //       0,   0,   0,   1,   0,   0,   0,   0,   0,
  //   //       0,   0,   0,   0,   1,   dt_, 0,   0,   0,
  //   //       0,   0,   0,   0,   0,   1,   0,   0,   0,
  //   //       0,   0,   0,   0,   0,   0,   1,   dt_, 0,
  //   //       0,   0,   0,   0,   0,   0,   0,   1,   0,
  //   //       0,   0,   0,   0,   0,   0,   0,   0,   1;
  //   // // clang-format on

  //   Eigen::MatrixXd f(11, 11);
  //   f.setIdentity();
  //   for (int i = 0; i < 3; ++i) {
  //       int base = 2 * i;
  //       f(base, base + 1) = dt_;
  //   }
  //   // yaw 的雅可比矩阵
  //   f(6, 7) = dt_;
  //   f(6, 8) = 0.5 * dt_ * dt_;
  //   f(6, 9) = (1.0 / 6.0) * dt_ * dt_ * dt_;
  //   f(7, 8) = dt_;
  //   f(7, 9) = 0.5 * dt_ * dt_;
  //   f(8, 9) = dt_;
  //   // r 的雅可比矩阵
  //   f(10,10) = 1;
  //   return f;
  // };

  auto j_f_a = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(10, 10);
    f.setIdentity();
    for (int i = 0; i < 3; ++i) {
        int base = 2 * i;
        f(base, base + 1) = dt_;
    }
    // yaw 的雅可比矩阵
    f(6, 7) = dt_;
    f(6, 8) = 0.5 * dt_ * dt_;
    f(7, 8) = dt_;
    // r 的雅可比矩阵
    f(9,9) = 1;
    return f;
  };

  // h - Observation function
  // auto h = [](const Eigen::VectorXd & x) {
  //   Eigen::VectorXd z(4);
  //   double xc = x(0), yc = x(2), yaw = x(6), r = x(10);
  //   z(0) = xc - r * cos(yaw);  // xa
  //   z(1) = yc - r * sin(yaw);  // ya
  //   z(2) = x(4);               // za
  //   z(3) = x(6);               // yaw
  //   return z;
  // };

  auto h_a = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(9);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(4);               // za
    z(3) = x(6);               // yaw
    return z;
  };

  // J_h - Jacobian of obs
  // auto j_h = [](const Eigen::VectorXd & x) {
  //   Eigen::MatrixXd h(4, 11);
  //   double yaw = x(6), r = x(10);
  //   // clang-format off
  //   //    xc   v_xc yc   v_yc za   v_za yaw          v_yaw a_yaw aa_yaw     r
  //   h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw),  0,    0,    0,     -cos(yaw),
  //         0,   0,   1,   0,   0,   0,   -r*cos(yaw), 0,    0,    0,     -sin(yaw),
  //         0,   0,   0,   0,   1,   0,   0,              0,    0,    0,      0,
  //         0,   0,   0,   0,   0,   0,   1,              0,    0,    0,      0;
  //   // clang-format on
  //   return h;
  // };

  auto j_h_a = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 10);
    double yaw = x(6), r = x(9);
    // clang-format off
    //    xc   v_xc yc   v_yc za   v_za yaw          v_yaw  a_yaw    r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw),  0,    0,    -cos(yaw),
          0,   0,   1,   0,   0,   0,   -r*cos(yaw), 0,    0,    -sin(yaw),
          0,   0,   0,   0,   1,   0,   0,              0,    0,    0,
          0,   0,   0,   0,   0,   0,   1,              0,    0,    0;
    // clang-format on
    return h;
  };

  // update_Q - process noise covariance matrix
  s2qxyz_max_ = declare_parameter("ekf.sigma2_q_xyz_max", 0.1);
  s2qxyz_min_ = declare_parameter("ekf.sigma2_q_xyz_min", 0.05);
  s2qyaw_max_ = declare_parameter("ekf.sigma2_q_yaw_max", 10.0);
  s2qyaw_min_ = declare_parameter("ekf.sigma2_q_yaw_min", 0.01);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 80.0);
  
  // 测试 加加速度恒定模型 用参数
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 0.05);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 7.0);

  // auto u_q = [this](const Eigen::VectorXd & innovation) {
  //   Eigen::MatrixXd q(11, 11);
  //   //* 创新向量 自适应参数
  //   // 提取平移部分 (xyz)
  //   Eigen::VectorXd innovation_xyz = innovation.segment(0, 6);  // 提取索引 [0, 1, 2, 3, 4, 5]
  //   // 提取旋转部分 (yaw)
  //   Eigen::VectorXd innovation_yaw = innovation.segment(6, 4);  // 提取索引 [6, 7, 8, 9]

  //   // 计算平移和旋转的范数
  //   double norm_xyz = innovation_xyz.norm();
  //   double norm_yaw = innovation_yaw.norm();

  //   // 动态调整因子
  //   double adaptive_factor_xyz = std::min(1.0, std::max(0.1, norm_xyz / 10.0));
  //   double adaptive_factor_yaw = std::min(1.0, std::max(0.1, norm_yaw / 10.0));

  //   double x, y;
    
  //   x = adaptive_factor_xyz * (s2qxyz_max_ - s2qxyz_min_) + s2qxyz_min_;
  //   y = adaptive_factor_yaw * (s2qyaw_max_ - s2qyaw_min_) + s2qyaw_min_;

  //   double t = dt_, r = s2qr_;
  //   double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
  //   // double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
  //   double q_y_y = pow(t, 6) / 36.0 * y,q_y_vy = pow(t, 5) / 12.0 * y, q_y_ay = pow(t, 4) / 6.0 * y, q_y_aay = pow(t, 3) / 6.0 * y;
  //   double q_vy_vy = pow(t, 4) / 4 * y, q_vy_ay = pow(t, 3) / 2 * y, q_vy_aay = pow(t, 2) * y;
  //   double q_ay_ay = pow(t, 2) * y, q_ay_aay = t * y, q_aay_aay = y;
  //   double q_r = pow(t, 4) / 4 * r;
  //   // clang-format off
  //   //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw    a_yaw   aa_yaw     r
  //   q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,       0,       0,        0,
  //         q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,       0,       0,        0,
  //         0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,       0,       0,        0,
  //         0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,       0,       0,        0,
  //         0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,       0,       0,        0,
  //         0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,       0,       0,        0,
  //         0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy,  q_y_ay,  q_y_aay,  0,
  //         0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy, q_vy_ay, q_vy_aay, 0,
  //         0,      0,      0,      0,      0,      0,      q_y_ay, q_vy_ay, q_ay_ay, q_ay_aay, 0,
  //         0,      0,      0,      0,      0,      0,      q_y_aay,q_vy_aay,q_ay_aay,q_aay_aay,0,
  //         0,      0,      0,      0,      0,      0,      0,      0,       0,       0,        q_r;
  //   // clang-format on
  //   // std::cout <<"Adaptive: " << adaptive_factor << std::endl;
  //   // std::cout << "s2gyaw: " << y <<std::endl;

  //   return q;
  // };

  auto u_q_a = [this](const Eigen::VectorXd & innovation) {
    Eigen::MatrixXd q(10, 10);
    //* 创新向量 自适应参数
    // 提取平移部分 (xyz)
    Eigen::VectorXd innovation_xyz = innovation.segment(0, 6);  // 提取索引 [0, 1, 2, 3, 4, 5]
    // 提取旋转部分 (yaw)
    Eigen::VectorXd innovation_yaw = innovation.segment(6, 3);  // 提取索引 [6, 7, 8]

    // 计算平移和旋转的范数
    double norm_xyz = innovation_xyz.norm();
    double norm_yaw = innovation_yaw.norm();

    // 动态调整因子
    double adaptive_factor_xyz = std::min(1.0, std::max(0.1, norm_xyz / 10.0));
    double adaptive_factor_yaw = std::min(1.0, std::max(0.1, norm_yaw / 10.0));

    double x, y;
    
    x = adaptive_factor_xyz * (s2qxyz_max_ - s2qxyz_min_) + s2qxyz_min_;
    y = adaptive_factor_yaw * (s2qyaw_max_ - s2qyaw_min_) + s2qyaw_min_;

    double t = dt_, r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    // double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_y_y = pow(t, 6) / 36.0 * y,q_y_vy = pow(t, 5) / 12.0 * y, q_y_ay = pow(t, 4) / 6.0 * y;
    double q_vy_vy = pow(t, 4) / 4 * y, q_vy_ay = pow(t, 3) / 2 * y;
    double q_ay_ay = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw    a_yaw    r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,       0,       0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,       0,       0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,       0,       0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,       0,       0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,       0,       0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,       0,       0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy,  q_y_ay,  0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy, q_vy_ay, 0,
          0,      0,      0,      0,      0,      0,      q_y_ay, q_vy_ay, q_ay_ay, 0,
          0,      0,      0,      0,      0,      0,      0,      0,       0,       q_r;
    // clang-format on
    // std::cout << "norm_yaw: " << norm_yaw <<std::endl;
    
    // std::cout <<"Adaptive: " << adaptive_factor_yaw << std::endl;
    // std::cout << "s2gyaw: " << y <<std::endl;

    return q;
  };


  // update_R - measurement noise covariance matrix
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double x = r_xyz_factor;
    r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
    return r;
  };

  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 10> p0;
  p0.setIdentity();
  
  // tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
  tracker_->ekf = ExtendedKalmanFilter{f_a, h_a, j_f_a, j_h_a, u_q_a, u_r, p0};


  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->tracker_state = Tracker::LOST; 
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return;
    });

  // Change target service
  change_target_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/change", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->tracker_state = Tracker::CHANGE_TARGET;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Target change!");
      return;
    });

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  
  //! Tracker 第四部分 订阅与发布端
  // 速度相关订阅
  velocity_sub_ = this->create_subscription<auto_aim_interfaces::msg::Velocity>(
    "/current_velocity", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
    std::bind(&ArmorTrackerNode::velocityCallback, this, std::placeholders::_1));

  // receive 订阅
  receive_sub_ = this->create_subscription<auto_aim_interfaces::msg::Receive>(
    "/tracker/receive", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
    std::bind(&ArmorTrackerNode::receiveCallback, this, std::placeholders::_1));

  // detector 延迟 订阅
  detctor_sub_= this->create_subscription<auto_aim_interfaces::msg::AllLatency>(
    "/detector/Latency", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
    std::bind(&ArmorTrackerNode::latencyCallback, this, std::placeholders::_1));

  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);



  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS());

  send_pub_ = this->create_publisher<auto_aim_interfaces::msg::Send>(
    "/tracker/send", rclcpp::SensorDataQoS());

  tracker_latency_pub_ = this->create_publisher<auto_aim_interfaces::msg::AllLatency>(
    "/tracker/latency", rclcpp::SensorDataQoS());

  test_pub_ = this->create_publisher<auto_aim_interfaces::msg::Test>(
    "/test", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;

  trackered_armor_marker_.ns = "trackered_armor";
  trackered_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  trackered_armor_marker_.scale.x = 0.03;
  trackered_armor_marker_.scale.z = 0.125;
  trackered_armor_marker_.color.a = 1.0;
  trackered_armor_marker_.color.g = 1.0;

  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  // aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}

//! 速度回调,在这里给的速度初始值
void ArmorTrackerNode::velocityCallback(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg){

  gaf_solver->init(velocity_msg);

}

void ArmorTrackerNode::receiveCallback(const auto_aim_interfaces::msg::Receive::SharedPtr receive_msg){
  gaf_solver->initReceive(receive_msg);
}

void ArmorTrackerNode::latencyCallback(const auto_aim_interfaces::msg::AllLatency::SharedPtr detector_msg){
  gaf_solver->initLatency(detector_msg);
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  auto tracker_latency_start = this->get_clock()->now();

  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
              Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                max_armor_distance_;
      }),
    armors_msg->armors.end());

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  auto_aim_interfaces::msg::Send send_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  send_msg.header.stamp = time;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // 测试通道
  target0_yaw = 0;


  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

    tracker_->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = 2;
      // Fill target message
      const auto & state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.a_yaw = state(8);

      target_msg.radius_1 = state(9);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.dz = tracker_->dz;

      // v_yaw 单位换算到 rpm
      target_msg.v_yaw_rpm = (target_msg.v_yaw / (2 * M_PI)) * 60;
      // v_yaw 单位换算到 degree/s
      target_msg.v_yaw_degree = target_msg.v_yaw * (180.0 / M_PI);

      
      //* 弹道解算
      // 算出来的 pitch与yaw就是 云台要去的yaw
      send_pitch=0, send_yaw=0, aim_x=0, aim_y=0, aim_z=0;  // 之所以要重置为0 是因为在弹道解算中会迭代计算
      auto msg = std::make_shared<auto_aim_interfaces::msg::Target>(target_msg);
      gaf_solver->autoSolveTrajectory(send_pitch, send_yaw, aim_x, aim_y, aim_z, msg, target0_yaw);

      send_msg.position.x = aim_x;
      send_msg.position.y = aim_y;
      send_msg.position.z = aim_z;
      send_msg.v_yaw = target_msg.v_yaw;
      send_msg.pitch = send_pitch;
      send_msg.yaw = send_yaw;

      // 开火指令
      // 下面的仍然是 拉姆达表达式 [&] 按引用捕获外部作用域中的所有变量。这意味着在 Lambda 内部可以直接访问和修改外部变量
      send_msg.is_fire = gaf_solver->shouldFire(send_msg.yaw, gaf_solver->receive_yaw);




    } else if (tracker_->tracker_state == Tracker::CHANGE_TARGET) {
      target_msg.tracking = false;
    }
  }
  
  last_time_ = time;

  // tracker 延迟计算
  auto tracker_latency_end = this->get_clock()->now();
  auto tracker_latency_dt = tracker_latency_end - tracker_latency_start;

  auto_aim_interfaces::msg::AllLatency tracker_latency_msg;
  tracker_latency_msg.header.stamp = time;
  tracker_latency_msg.tracker_latency = static_cast<int>(tracker_latency_dt.seconds() * 1000);

  // 测试通道
  auto_aim_interfaces::msg::Test target0_yaw_;
  target0_yaw_.header.stamp = time; 
  target0_yaw_.yaw = target0_yaw; 
  target0_yaw_.d_yaw = gaf_solver->d_yaw;

  test_pub_->publish(target0_yaw_);



  // 发布 msg
  send_pub_->publish(send_msg);

  target_pub_->publish(target_msg);

  tracker_latency_pub_ -> publish(tracker_latency_msg);

  publishMarkers(target_msg);

}



























void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;
  aiming_point_.header = target_msg.header;
  trackered_armor_marker_.header = target_msg.header;
  
  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;

    trackered_armor_marker_.action = armor_marker_.action;
    trackered_armor_marker_.scale.y = armor_marker_.scale.y;

    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.pose.position.x = aim_x;
    aiming_point_.pose.position.y = aim_y;
    aiming_point_.pose.position.z = aim_z;

    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);

      if (i == 0) {
        trackered_armor_marker_.id = i;
        trackered_armor_marker_.pose.position = p_a;
        trackered_armor_marker_.pose.orientation = tf2::toMsg(q);
      } else {
        armor_marker_.id = i;
        armor_marker_.pose.position = p_a;
        armor_marker_.pose.orientation = tf2::toMsg(q);
      }
      marker_array.markers.emplace_back(armor_marker_);
      marker_array.markers.emplace_back(trackered_armor_marker_);

    }
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    trackered_armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;  // 删除 trackered_armor
    aiming_point_.action = visualization_msgs::msg::Marker::DELETEALL;  // 删除 aiming_point_
    marker_array.markers.emplace_back(armor_marker_);
    marker_array.markers.emplace_back(trackered_armor_marker_);

  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_array.markers.emplace_back(aiming_point_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)