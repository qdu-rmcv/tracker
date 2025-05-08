#include "armor_solve/solve_node.hpp"

//STD

#include <algorithm>
#include <auto_aim_interfaces/msg/detail/send__struct.hpp>
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>
#include <iostream>

namespace rm_auto_aim
{
ArmorSolveNode::ArmorSolveNode(const rclcpp::NodeOptions & options)
: Node("armor_solve", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting SolveNode!");

    // 用于弹道解算的k值具体计算在git
    k = this->declare_parameter("solve.k", 0.092);
    bias_time = this->declare_parameter("solve.bias_time", 100);
    s_bias = this->declare_parameter("solve.s_bias", 0.18375);
    z_bias = this->declare_parameter("solve.z_bias", 0.196);

    // 
    v_top = this->declare_parameter("solve.v_top", 24);

    // Publisher

    send_pub_ = this->create_publisher<auto_aim_interfaces::msg::Send>(
        "/solve/send", rclcpp::SensorDataQoS());

    // Subscriber
    receive_sub_ = this->create_subscription<auto_aim_interfaces::msg::Receive>(
        "/solve/receive", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
        std::bind(&ArmorSolveNode::receiveCallback, this, std::placeholders::_1));

    velocity_sub_ = this->create_subscription<auto_aim_interfaces::msg::Velocity>(
        "/current_velocity", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
        std::bind(&ArmorSolveNode::velocityCallback, this, std::placeholders::_1));
    
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/solve/target", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
        std::bind(&ArmorSolveNode::targetCallback, this, std::placeholders::_1));


    // marker
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/solve/marker", 10);
}

void ArmorSolveNode::velocityCallback(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
{

    solve_ -> initVelocity(velocity_msg);

}

void ArmorSolveNode::receiveCallback(const auto_aim_interfaces::msg::Receive::SharedPtr receive_msg)
{
    solve_ -> initReceive(receive_msg);
}

void ArmorSolveNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg)
{
    auto solve_latency_start = this->get_clock()->now();

    // Init message
    // tar_yaw = target_msg->yaw;

    // calculate position for 2,3,4

    // selectArmor fast low

    // calculate pitch yaw

    // publish
}



}  // namespace rm_auto_aim