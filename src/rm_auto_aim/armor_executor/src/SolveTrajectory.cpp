/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


// STD
#include <algorithm>
#include <auto_aim_interfaces/msg/detail/receive__struct.hpp>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
#include <iostream>


#include "armor_executor/SolveTrajectory.hpp"

namespace rm_auto_aim
{
SolveTrajectory::SolveTrajectory(const float &k, const int &bias_time, const float &s_bias, const float &z_bias, const float &fire_k)
: k(k), bias_time(bias_time), s_bias(s_bias), z_bias(z_bias), fire_k(fire_k)
{}


void SolveTrajectory::init(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
{   
    if (!std::isnan(velocity_msg->velocity)) { // 使用 std::isnan 检查是否为 NAN
        current_v = velocity_msg->velocity;
        // current_v = 25.5; // debug
    } else if (current_v <= 5) {
    current_v = 25.5;
    } else {
        current_v = 25.5; // 默认值
        std::cout << "电控没有传速度,默认设置成25.5了" << std::endl;
    }
}

void SolveTrajectory::initReceive(const auto_aim_interfaces::msg::Receive::SharedPtr receive_msg)
{
    // if (std::isnan(receive_msg->yaw)) {
    //     RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "receive_msg 为 空");
    // }

    receive_pitch = receive_msg->pitch;
    receive_yaw = receive_msg->yaw;
    receive_roll = receive_msg->roll;
}

void SolveTrajectory::initLatency(const auto_aim_interfaces::msg::AllLatency::SharedPtr detector_msg){
    if (detector_msg->detector_latency > 150) {
        det_latency = 100;
    } else {
        det_latency = detector_msg->detector_latency;
    }
}

//! 单方向无空气阻力弹道模型
/**
 * @brief 初始化时间 详情去看 [[无空气阻力单方向 弹道解算模型]]
 * 
 * @param s_bias 枪口前推偏置
 * @param z_bias z偏置
 * @param current_v 弹速
 * @param msg 装甲板信息
 */
void SolveTrajectory::solveTimeInit(float s_bias, float z_bias, float current_v, const auto_aim_interfaces::msg::Target::SharedPtr& msg)
{
    float s = std::sqrt((msg->position.x) * (msg->position.x) + (msg->position.y) * (msg->position.y)) - s_bias;
    float z = msg->position.z + z_bias;

    // 整理之后的二次项系数
    float part_a = 0.5 * GRAVITY * s * s / (current_v * current_v);
    // 值呢管理之后的根号下判别式
    float part_b = s * s - 4 *part_a * (z + part_a);

    // tan(theta)  
    float ttheta = 0;
    // 弹道解算的角度
    float theata = 0;

    // 判别式子不合理排除
    if (part_b < 0) {
        std::cout << "初始化解算失败01" << std::endl;
        ftime = 0.11f;
    } else {
    float ttheta_1 = (s + std::sqrt(part_b)) / (2 * part_a);
    float ttheta_2 = (s - std::sqrt(part_b)) / (2 * part_a);

    // float theta_1 = std::atan(ttheta_1);
    // float theta_2 = std::atan(ttheta_2);

    // std::cout << "theta_1: " << theta_1 << std::endl;
    // std::cout << "theta_2: " << theta_2 << std::endl; 
    // float ftime_1 = calculateFlyTime(s, current_v, theta_1);
    // float ftime_2 = calculateFlyTime(s, current_v, theta_2);

    // std::cout << "ftime_1: " << ftime_1 << std::endl;
    // std::cout << "ftime_2: " << ftime_2 << std::endl;

        if (ttheta_1 > 0 && ttheta_2 > 0) {
            // 取最小值,为底抛物线解
            ttheta = std::min(ttheta_1, ttheta_2);
        }
    }

    theata = std::atan(ttheta);

    ftime = calculateFlyTime(s, current_v, theata);
}


bool SolveTrajectory::shouldFire(const float send_yaw, const float receive_yaw){
    d_yaw = std::abs(send_yaw - receive_yaw);
    fire_k = 0.002;
    is_fire = false;
    if(d_yaw < fire_k){
        is_fire = true;
    }

    return is_fire;
}

/**
 * @brief 根据距离、速度和角度计算飞行时间
 * @param s 距离 (m)
 * @param v 初速度 (m/s)
 * @param angle 发射角度 (rad)
 * @return 飞行时间 (s)
 */
float SolveTrajectory::calculateFlyTime(float s, float v, float angle) {
    return (std::exp(k * s) - 1) / (k * v * std::cos(angle));
}

//! 单方向空气阻力弹道模型
/*
@brief 简单物理模型自己去推导
@param s:m 距离
@param v:m/s 速度
@param angle:rachouxiang
@return z:m
*/
float SolveTrajectory::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z,t;
    //t为给定v与angle时的飞行时间
    t = calculateFlyTime(s, v, angle);

    //z为给定v与angle时的高度
    z = (float)(v * std::sin(angle) * t - GRAVITY * t * t / 2);

    return z;
}

//! 判断是否开火
/**
 * @brief 一种线性预测
 * 
 * @param tmp_yaw 四块装甲板
 * @param v_yaw yaw速度
 * @param timeDelay 时间延迟
 * 
 * @return true 
 * @return false 
 */
// bool SolveTrajectory::shouldFire(const auto_aim_interfaces::msg::Target::SharedPtr& msg) {
//     for (int i = 0; i < msg->armors_num; i++){
//         tar_position[i].yaw
//     }
//     //? 为何减去2PI 打击旋转一圈之后的
//     return std::fabs((tmp_yaw + v_yaw * timeDelay) - 2 * PI) < 0.001;
// }

//! pitch轴解算
/** 
* @brief pitch轴解算 一般全解算次数在20 - 27 之间,修改for范围
* @param s:m 距离
* @param z:m 高度
* @param v:m/s
* @return angle_pitch:rad
*/
float SolveTrajectory::pitchTrajectoryCompensation(float s, float z, float v) {
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
    // std::cout << "pitch解算数:" << num << std::endl;

    ftime = calculateFlyTime(s, v, angle_pitch);
    
    // std::cout << "ftime +: " << ff << std::endl;
    return angle_pitch;
}



//! 解算四块装甲板位置
/**
 * @brief 根据当前观测到的装甲板信息，计算出来所有装甲板位置
 * 
 * @param auto_aim_interfaces::msg::Target::SharedPtr& msg
 * @param use_1 标志
 * @param use_average_radius 是否使用平均半径
 */
void SolveTrajectory::calculateArmorPosition(const auto_aim_interfaces::msg::Target::SharedPtr& msg, bool use_1, bool use_average_radius, float target0_yaw) {

    std::vector<float> tmp_yaws; 

    min_yaw_in_cycle = std::numeric_limits<float>::max();   //?何用之有
    max_yaw_in_cycle = std::numeric_limits<float>::min();
    // 对每块装甲板
    for (int i = 0; i < msg->armors_num; i++) {
        // 计算 tmp_yaw,目标yaw换算,并且除以装甲板数量
        float tmp_yaw = tar_yaw + i * 2.0 * PI / msg->armors_num;
        tmp_yaws.push_back(tmp_yaw);
        min_yaw_in_cycle = std::min(min_yaw_in_cycle, tmp_yaw);
        max_yaw_in_cycle = std::max(max_yaw_in_cycle, tmp_yaw);

        // 半径
        float r;
        if (use_average_radius) {
            // 使用两个半径的平均值
            r = (msg->radius_1 + msg->radius_2) / 2;
        } else {
            // 使用r1或r2
            r = use_1 ? msg->radius_1 : msg->radius_2;
        }
        // 简单的三角函数计算,记住四块装甲板位置
        tar_position[i].x = msg->position.x - r * std::cos(tmp_yaw);
        tar_position[i].y = msg->position.y - r * std::sin(tmp_yaw);
        tar_position[i].z = msg->position.z;
        tar_position[i].yaw = tmp_yaw;
        use_1 = !use_1;

        // std::cout<<"ppp tar_position"<< tar_position[i].yaw <<std::endl;
    }
    target0_yaw = tar_position[0].yaw;
    // std::cout<<"yaw"<< tar_position[0].yaw <<std::endl;
}

//! 解算Pitch && Yaw
/**
 * @brief 
 * 
 * @param idx 最适合开获得装甲板号
 * @param auto_aim_interfaces::msg::Target::SharedPtr& msg 装甲板信息
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
std::pair<float, float> SolveTrajectory::calculatePitchAndYaw(int idx, const auto_aim_interfaces::msg::Target::SharedPtr& msg, float timeDelay, float s_bias, float z_bias, float current_v, bool use_target_center_for_yaw,float& aim_x, float& aim_y, float& aim_z) {
    
    // 对打击目标xyz进行线性预测,初步的落点
    aim_x = tar_position[idx].x  + msg->velocity.x * timeDelay;
    aim_y = tar_position[idx].y  + msg->velocity.y * timeDelay;
    aim_z = tar_position[idx].z;
    
    // 切换识别装甲板还是robt 中心
    float yaw_x = use_target_center_for_yaw ? msg->position.x : aim_x;
    float yaw_y = use_target_center_for_yaw ? msg->position.y : aim_y;

    //* 真正的 pitch轴 解算
    float send_pitch = pitchTrajectoryCompensation(std::sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - s_bias,
            aim_z + z_bias, current_v);
    
    // yaw轴解算
    float send_yaw = (float)(std::atan2(yaw_y, yaw_x));

    return std::make_pair(send_pitch, send_yaw);
}

int SolveTrajectory::selectArmor(const auto_aim_interfaces::msg::Target::SharedPtr& msg, bool select_by_min_yaw) {
    int selected_armor_idx = 0;

    float xc_odom = (float)(std::atan2(msg -> position.y, msg -> position.x));

    if (select_by_min_yaw) {
        // 选择枪管到目标装甲板yaw最小的那dz个装甲板
        float min_yaw_diff = std::fabs(xc_odom - tar_position[0].yaw);

        // std::cout << "tar_position[0].yaw " << tar_position[0].yaw << std::endl;
        // std::cout << "min_yaw_diff " << min_yaw_diff << std::endl;

        for (int i = 1; i < msg->armors_num; i++) {
            float temp_yaw_diff = std::fabs(xc_odom - tar_position[i].yaw);

            // std::cout << "temp_yaw_diff " << temp_yaw_diff << std::endl;

            if (temp_yaw_diff < min_yaw_diff) {
                min_yaw_diff = temp_yaw_diff;
                selected_armor_idx = i; 
            }
        }
    } else {
        // 选择离你的机器人最近的装甲板 
        float min_distance = std::sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        for (int i = 1; i < msg->armors_num; i++) {
            float distance = std::sqrt(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y);
            if (distance < min_distance) {
                min_distance = distance;
                selected_armor_idx = i;
            }
        }
    }
    return selected_armor_idx;
}

//! 最优开火指令
/**
 * @brief 先进行一次线性预测，然后根据预测的位置进行开火逻辑 
 * 
 * @param send_pitch 
 * @param send_yaw 
 * @param aim_x 
 * @param aim_y 
 * @param aim_z 
 * @param auto_aim_interfaces::msg::Target::SharedPtr& msg
 */
void SolveTrajectory::fireLogicIsTop(float& send_pitch, float& send_yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr& msg, float target0_yaw) {

    tar_yaw = msg->yaw;
    // int delay_time = bias_time + det_latency;
    int delay_time = bias_time;
    // 线性预测
    float timeDelay = delay_time/1000.0 + ftime;  // 改为 测距 所用时间 


    // // 线性预测
    // if (msg->a_yaw < 20 && msg->v_yaw < 3) {
    //     tar_yaw += msg->v_yaw * timeDelay + 0.5 * msg->a_yaw * timeDelay * timeDelay; 
    // } else {
    //     tar_yaw += msg->v_yaw * timeDelay;
    // }

    tar_yaw += msg->v_yaw * timeDelay;
    
    // std::cout << "tar_yaw: " << tar_yaw << std::endl; 
    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
    int idx = 0;
    // bool is_fire = false;

    // 对于 平衡步兵 
    if (msg->armors_num  == ARMOR_NUM_BALANCE) {
        calculateArmorPosition(msg , true, false, target0_yaw);
        idx = selectArmor(msg, true);
    // 对于 前哨站
    } else if (msg->armors_num == ARMOR_NUM_OUTPOST) {
        calculateArmorPosition(msg, false, true, target0_yaw);
        idx = selectArmor(msg, false);
        }
    // 对于普通装甲板 
    else {
        //* 解算装甲板位置 注意 use_1 以及 use_average_radius
        calculateArmorPosition(msg, false, false, target0_yaw);

        // 切换看应不应该开火
        // 找到四块装甲板中最适合开火的那一块 idx ,并且如果都不适合就以最后一块也就是当前追踪的装甲板为准
        idx = selectArmor(msg, false);
    }
    // std::cout<<"[SolveTrajectory] idx is: "<< idx <<std::endl;

    //std::cout << "pppp 1 aim_x " << tar_position[idx].x<<std::endl;
    //std::cout<<"pppp 1 [SolveTrajectory] aim_x is "<<aim_x<<std::endl;
    
    std::pair<float, float> pitch_and_yaw;

    //* 解算 pitch 和 yaw
    // if (std::fabs(msg->v_yaw) > 50.0f) {
    //     pitch_and_yaw = calculatePitchAndYaw(idx, msg, timeDelay, s_bias, z_bias, current_v, true, aim_x, aim_y, aim_z);
    //     // shouldFire(msg);
    // } else {
    //     pitch_and_yaw = calculatePitchAndYaw(idx, msg, timeDelay, s_bias, z_bias, current_v, false, aim_x, aim_y, aim_z);
    // }
    pitch_and_yaw = calculatePitchAndYaw(idx, msg, timeDelay, s_bias, z_bias, current_v, false, aim_x, aim_y, aim_z);


    // 使用 pitch_and_yaw 的值
    send_pitch = pitch_and_yaw.first;
    send_yaw = pitch_and_yaw.second;

}

/**
* @brief 根据最优决策得出被击打装甲板 自动解算弹道
* @param send_pitch:rad  传出pitch
* @param send_yaw:rad    传出yaw
* @param aim_x:传出aim_x  打击目标的x
* @param aim_y:传出aim_y  打击目标的y
* @param aim_z:传出aim_z  打击目标的z
*/
void SolveTrajectory::autoSolveTrajectory(float& send_pitch, float& send_yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr msg, float target0_yaw)
{
    // solveTimeInit(s_bias, z_bias, current_v,msg);
    // std::cout << "ftime -: " << ftime << std::endl;
    //* 优先开火逻辑
    fireLogicIsTop(send_pitch, send_yaw, aim_x, aim_y, aim_z, msg, target0_yaw);
}

// 从坐标轴正向看向原点，逆时针方向为正

}  // namespace rm_auto_aim
