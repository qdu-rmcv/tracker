#include "armor_solve/solve.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_auto_aim
{

void Solve::initVelocity(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
{   
    if (!std::isnan(velocity_msg->velocity)) { // 使用 std::isnan 检查是否为 NAN
        current_v = velocity_msg->velocity;
        // current_v = 25.5; // debug
    } else if (current_v <= 5 || current_v >= 30) {
    current_v = 24.0;
    RCLCPP_ERROR(rclcpp::get_logger("armor_solve"), "速度值太小或太大 ");
    } else {
        current_v = 24.0; // 默认值
        RCLCPP_ERROR(rclcpp::get_logger("armor_solve"), "速度值为 Nan ");
    }
}

void Solve::initReceive(const auto_aim_interfaces::msg::Receive::SharedPtr receive_msg)
{
    receive_pitch = receive_msg->pitch;
    receive_yaw = receive_msg->yaw;
    receive_roll = receive_msg->roll;
}

//! 解算四块装甲板位置
/**
 * @brief 根据当前观测到的装甲板信息，计算出来所有装甲板位置
 * 
 * @param auto_aim_interfaces::msg::Target::SharedPtr& msg
 * @param use_1 标志
 * @param use_average_radius 是否使用平均半径
 */
void Solve::calculateArmorPosition(const auto_aim_interfaces::msg::Target::SharedPtr msg, bool use_1, bool use_average_radius, float target0_yaw) {

    std::vector<float> tmp_yaws; 

    // float min_yaw_in_cycle = std::numeric_limits<float>::max();   //?何用之有
    // float max_yaw_in_cycle = std::numeric_limits<float>::min();
    // // 对每块装甲板
    // for (int i = 0; i < msg->armors_num; i++) {
    //     // 计算 tmp_yaw,目标yaw换算,并且除以装甲板数量
    //     float tmp_yaw = tar_yaw + i * 2.0 * PI / msg->armors_num;
    //     tmp_yaws.push_back(tmp_yaw);
    //     min_yaw_in_cycle = std::min(min_yaw_in_cycle, tmp_yaw);
    //     max_yaw_in_cycle = std::max(max_yaw_in_cycle, tmp_yaw);

    //     // 半径
    //     float r;
    //     if (use_average_radius) {
    //         // 使用两个半径的平均值
    //         r = (msg->radius_1 + msg->radius_2) / 2;
    //     } else {
    //         // 使用r1或r2
    //         r = use_1 ? msg->radius_1 : msg->radius_2;
    //     }
    //     // 简单的三角函数计算,记住四块装甲板位置
    //     tar_position[i].x = msg->position.x - r * std::cos(tmp_yaw);
    //     tar_position[i].y = msg->position.y - r * std::sin(tmp_yaw);
    //     tar_position[i].z = msg->position.z;
    //     tar_position[i].yaw = tmp_yaw;
    //     use_1 = !use_1;

    //     // std::cout<<"ppp tar_position"<< tar_position[i].yaw <<std::endl;
    // }
    target0_yaw = tar_position[0].yaw;
    std::cout<<"yaw"<< tar_position[0].yaw <<std::endl;
}

}   //namespace rm_auto_aim


