#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstddef>
#include <vector>
#include <iostream>

#include "buff_executor/BuffSolveTrajectory.hpp"

namespace rm_buff 
{
BuffSolveTrajectory::BuffSolveTrajectory(const float &k, const int &bias_time, const float &s_bias, const float &z_bias)
: k(k), bias_time(bias_time), s_bias(s_bias), z_bias(z_bias)
{}
void BuffSolveTrajectory::init(const buff_interfaces::msg::Velocity::SharedPtr velocity_msg)
{   
    if (!std::isnan(velocity_msg->velocity)) { // 使用 std::isnan 检查是否为 NAN
        current_v = velocity_msg->velocity;
        // current_v = 25.5; // debug
    } else {
        current_v = 25.5; // 默认值
        std::cout << "电控没有给你传速度,我给设置成25.5了" << std::endl;
    }
}

//! 单方向空气阻力弹道模型
/*
@brief 简单物理模型自己去推导
@param s:m 距离
@param v:m/s 速度
@param angle:rachouxiang
@return z:m
*/
float BuffSolveTrajectory::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z, t;
    //t为给定v与angle时的飞行时间
    t = (float)((std::exp(k * s) - 1) / (k * v * std::cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * std::sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}


//! pitch轴解算
/** 
* @brief pitch轴解算 一般全解算次数在20 - 27 之间,修改for范围
* @param s:m 距离
* @param z:m 高度
* @param v:m/s
* @return angle_pitch:rad
*/
float BuffSolveTrajectory::pitchTrajectoryCompensation(float s, float z, float v) {
    // 初始
    float z_temp = z;
    float angle_pitch = 0.0f;
    int num = 0;

    // 迭代求解 send_pitch 注意看图
    for (int i = 0; i < 22; i++) {
        angle_pitch = std::atan2(z_temp, s);
        //* 单方向空气阻力模型
        float z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        float dz = 0.3f * (z - z_actual);
        z_temp += dz;
        num++;

        if (std::fabs(dz) < 0.00001f) {
            break;
        }
    }
    std::cout << "pitch解算数:" << num << std::endl;

    return angle_pitch;
}
//! 解算Pitch && Yaw
/**
 * @brief 
 * 
 * @param idx 最适合开获得装甲板号
 * @param buff_interface::msg::Target::SharedPtr& msg 装甲板信息
 * @param timeDelay 延迟时间
 * @param s_bias 枪口前推偏置
 * @param z_bias z偏置
 * @param current_v 弹速
 * @param use_target_center_for_yaw 是否使用角度选板逻辑
 * @param aim_x x打击落点
 * @param aim_y y打击落点
 * @param aim_z z打击落点
 * @return std::pair<float, float> 
 */
std::pair<float, float> BuffSolveTrajectory::calculatePitchAndYaw(float timeDelay, float s_bias, float z_bias, float current_v,float& target_x, float& target_y, float& target_z,const buff_interfaces::msg::Rune::SharedPtr msg) {
    
    float yaw_x = msg->position.x;
    float yaw_y = msg->position.y;

    float distance_xy = std::sqrt(target_x * target_x + target_y * target_y);

    //* 真正的 pitch轴 解算
    float send_pitch = pitchTrajectoryCompensation(distance_xy - s_bias,target_z + z_bias, current_v);
    std::cout << "send_pitch:" << send_pitch << std::endl;
    
    // yaw轴解算
    float send_yaw = (float)(std::atan2(yaw_y, yaw_x));

    return std::make_pair(send_pitch, send_yaw);
}

//! 解算xyz
void BuffSolveTrajectory::calculateBuffPosition(const buff_interfaces::msg::Rune::SharedPtr msg, float& target_x, float& target_y, float& target_z) {
    // TODO: obtain real-time timestamp in practice
    uint64_t self_timestamp = 1234502;  // placeholder ms
    uint64_t cap_timestamp = 1234500;
    uint16_t t_offset = 1233;

    float delay = 0.3f;                 // prediction delay in seconds

    float t0 = static_cast<float>(cap_timestamp + t_offset) / 1000.0f;
    float t1 = static_cast<float>(self_timestamp + t_offset) / 1000.0f + delay;

    float xc = msg->position.x;
    float yc = msg->position.y;
    float zc = msg->position.z;
    float theta = msg->theta;
    float a = msg->a;
    float b = msg->b;
    float w = msg->w;
    float r = msg->r;

    if (w == 0.0f) {
        theta += b * (t1 - t0);
    } else {
        theta += a / w * (std::cos(w * t0) - std::cos(w * t1)) + b * (t1 - t0);
    }

    float norm_xy = std::sqrt(xc * xc + yc * yc);
    target_x = xc + r * (std::sin(theta) * yc / norm_xy);
    target_y = yc + r * (-std::sin(theta) * xc / norm_xy);
    target_z = zc + r * std::cos(theta);
}


/**
* @brief 解算弹道
* @param send_pitch:rad  传出pitch
* @param send_yaw:rad    传出yaw
* @param aim_x:传出aim_x  打击目标的x
* @param aim_y:传出aim_y  打击目标的y
* @param aim_z:传出aim_z  打击目标的z
*/
void BuffSolveTrajectory::autoBuffSolveTrajectory(float& send_pitch, float& send_yaw,const buff_interfaces::msg::Rune::SharedPtr msg,
                                            float& target_x, float& target_y, float& target_z)
{
    // 线性预测
    float timeDelay = bias_time/1000.0 + fly_time;

    calculateBuffPosition(msg,target_x, target_y, target_z);

    //* 解算pitch和yaw
    auto pitch_and_yaw = calculatePitchAndYaw(timeDelay, s_bias, z_bias, current_v, target_x, target_y, target_z, msg);
    //std::cout<<"pppppp6 aim_x "<< aim_x <<std::endl;
    send_pitch = pitch_and_yaw.first;
    send_yaw = pitch_and_yaw.second;
}

}  // namespace rm_buff