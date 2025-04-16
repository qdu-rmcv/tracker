/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


// STD
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
#include <iostream>



#include "armor_executor/SolveTrajectory.hpp"

namespace rm_auto_aim
{
SolveTrajectory::SolveTrajectory(const float &k, const int &bias_time, const float &s_bias, const float &z_bias)
: k(k), bias_time(bias_time), s_bias(s_bias), z_bias(z_bias)
{}


void SolveTrajectory::init(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
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
float SolveTrajectory::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z, t;
    //t为给定v与angle时的飞行时间
    t = (float)((std::exp(k * s) - 1) / (k * v * std::cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * std::sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}


//! 完整弹道模型
/*
@brief 完整弹道模型,事实上影响不大
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float SolveTrajectory::completeAirResistanceModel(float s, float v, float angle)
{
    // TODO: Implement complete air resistance model
    return 0.0f;
}


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

    return angle_pitch;
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
bool SolveTrajectory::shouldFire(float tmp_yaw, float v_yaw, float timeDelay) {

// std::cout << "tmp_yaw: " << tmp_yaw << std::endl;
// std::cout << "v_yaw: " << v_yaw << std::endl;
// std::cout << "timeDelay: " << timeDelay << std::endl;
// std::cout << "求和 " << tmp_yaw + v_yaw * timeDelay << std::endl;

    //? 为何减去2PI 打击旋转一圈之后的
    return std::fabs((tmp_yaw + v_yaw * timeDelay) - 2 * PI) < 0.001;
}

//! 解算四块装甲板位置
/**
 * @brief 根据当前观测到的装甲板信息，计算出来所有装甲板位置
 * 
 * @param auto_aim_interfaces::msg::Target::SharedPtr& msg
 * @param use_1 标志
 * @param use_average_radius 是否使用平均半径
 */
void SolveTrajectory::calculateArmorPosition(const auto_aim_interfaces::msg::Target::SharedPtr& msg, bool use_1, bool use_average_radius) {

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

        //std::cout<<"ppp x"<< tar_position[i].x <<std::endl;
    }
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

    if (select_by_min_yaw) {
        // 选择枪管到目标装甲板yaw最小的那dz个装甲板
        float min_yaw_diff = fabs(msg->yaw - tar_position[0].yaw);
        for (int i = 1; i < msg->armors_num; i++) {
            float temp_yaw_diff = fabs(msg->yaw - tar_position[i].yaw);
            if (temp_yaw_diff < min_yaw_diff) {
                min_yaw_diff = temp_yaw_diff;
                selected_armor_idx = i;
            }
        }
    } else {
        // 选择离你的机器人最近的装甲板
        float min_distance = std::numeric_limits<float>::max();
        for (int i = 0; i < msg->armors_num; i++) {
            float distance = std::sqrt(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y + tar_position[i].z * tar_position[i].z);
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
void SolveTrajectory::fireLogicIsTop(float& send_pitch, float& send_yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr& msg) {

    tar_yaw = msg->yaw;
    // // 线性预测
    float timeDelay = bias_time/1000.0 + fly_time;


    // 线性预测
    // float timeDelay = bias_time/1000.0 + fly_time;
    // tar_yaw += msg->v_yaw * timeDelay;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
    int idx = 0;
    bool is_fire = false;

    // 对于 平衡步兵 
    if (msg->armors_num  == ARMOR_NUM_BALANCE) {
        calculateArmorPosition(msg , true, false);
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            float tmp_yaw = tmp_yaws[i];
            if (shouldFire(tmp_yaw, msg->v_yaw, timeDelay)) {
                is_fire = true;
                idx = i;
                if (fireCallback) {
                    fireCallback(is_fire);
                }
                break;
            }
        }  
    // 对于 前哨站
    } else if (msg->armors_num == ARMOR_NUM_OUTPOST) {
        calculateArmorPosition(msg, false, true);
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            float tmp_yaw = tmp_yaws[i];
            if (shouldFire(tmp_yaw, msg->v_yaw, timeDelay)) {
                is_fire = true;
                idx = i;
                if (fireCallback) {
                    fireCallback(is_fire);
                }
                break;
            }
        }
    // 对于普通装甲板  
    } else {
        //* 解算装甲板位置 注意 use_1 以及 use_average_radius
        calculateArmorPosition(msg, false, false);

        // 切换看应不应该开火
        // 找到四块装甲板中最适合开火的那一块 idx ,并且如果都不适合就以最后一块也就是当前追踪的装甲板为准
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            idx = selectArmor(msg, true);
        }
    }


    //std::cout << "pppp 1 aim_x " << tar_position[idx].x<<std::endl;
    //std::cout<<"pppp 1 [SolveTrajectory] aim_x is "<<aim_x<<std::endl;
    
    //* 解算pitch和yaw
    auto pitch_and_yaw = calculatePitchAndYaw(idx, msg, timeDelay , s_bias, z_bias, current_v, false,aim_x,aim_y,aim_z);
    //std::cout<<"pppppp6 aim_x "<< aim_x <<std::endl;
    send_pitch = pitch_and_yaw.first;
    send_yaw = pitch_and_yaw.second;
}

// void SolveTrajectory::fireLogicOnlycentry(float& send_pitch, float& send_yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr& msg){
//     tar_yaw = msg->yaw;
//     // // 线性预测
//     float timeDelay = bias_time/1000.0 + fly_time;


//     // 线性预测
//     // float timeDelay = bias_time/1000.0 + fly_time;
//     // tar_yaw += msg->v_yaw * timeDelay;

//     //计算四块装甲板的位置
//     //装甲板id顺序，以四块装甲板为例，逆时针编号
//     //      2
//     //   3     1
//     //      0
//     int idx = 0;
//     bool is_fire = false;

//     // 对于 平衡步兵 
//     if (msg->armors_num  == ARMOR_NUM_BALANCE) {
//         calculateArmorPosition(msg , true, false);
//         for (size_t i = 0; i < tmp_yaws.size(); i++) {
//             float tmp_yaw = tmp_yaws[i];
//             if (shouldFire(tmp_yaw, msg->v_yaw, timeDelay)) {
//                 is_fire = true;
//                 idx = i;
//                 if (fireCallback) {
//                     fireCallback(is_fire);
//                 }
//                 break;
//             }
//         }  
//     // 对于 前哨站
//     } else if (msg->armors_num == ARMOR_NUM_OUTPOST) {
//         calculateArmorPosition(msg, false, true);
//         for (size_t i = 0; i < tmp_yaws.size(); i++) {
//             float tmp_yaw = tmp_yaws[i];
//             if (shouldFire(tmp_yaw, msg->v_yaw, timeDelay)) {
//                 is_fire = true;
//                 idx = i;
//                 if (fireCallback) {
//                     fireCallback(is_fire);
//                 }
//                 break;
//             }
//         } 
//     // 对于普通装甲板  
//     } else {

//         //* 解算装甲板位置 注意 use_1 以及 use_average_radius
//         calculateArmorPosition(msg, false, false);

//         // 切换看应不应该开火
//         // 找到四块装甲板中最适合开火的那一块 idx ,并且如果都不适合就以最后一块也就是当前追踪的装甲板为准
//         for (size_t i = 0; i < tmp_yaws.size(); i++) {
//             float tmp_yaw = tmp_yaws[i];

//             //* 判断是否开火
//             if (shouldFire(tmp_yaw, msg->v_yaw, timeDelay)) {

//                 is_fire = true;
//                 idx = i;
//                 if (fireCallback) {
//                     fireCallback(is_fire);
//                 }
//                 break;
//             }
//         }  
//     }

//     //std::cout << "pppp 1 aim_x " << tar_position[idx].x<<std::endl;
//     //std::cout<<"pppp 1 [SolveTrajectory] aim_x is "<<aim_x<<std::endl;
    
//     //* 解算pitch和yaw
//     auto pitch_and_yaw = calculatePitchAndYaw(idx, msg, timeDelay, s_bias, z_bias, current_v, true,aim_x,aim_y,aim_z);
//     //std::cout<<"pppppp6 aim_x "<< aim_x <<std::endl;
//     send_pitch = pitch_and_yaw.first;
//     send_yaw = pitch_and_yaw.second;
// }


// void SolveTrajectory::fireLogicDefault(float& send_pitch, float& send_yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr& msg) {

//     // 线性预测
//     float timeDelay = bias_time/1000.0 + fly_time;
//     tar_yaw += msg->v_yaw * timeDelay;

//     //计算四块装甲板的位置
//     //装甲板id顺序，以四块装甲板为例，逆时针编号
//     //      2
//     //   3     1
//     //      0
//     int idx = 0;
//     bool is_fire = false;
//     if (msg->armors_num  == ARMOR_NUM_BALANCE) {
//         calculateArmorPosition(msg, true, false);
//         for (size_t i = 0; i < tmp_yaws.size(); i++) {
//             idx = selectArmor(msg, true);
//             is_fire = tmp_yaws[idx] >= min_yaw_in_cycle && tmp_yaws[idx] <= max_yaw_in_cycle;
//             if (fireCallback) {
//                 fireCallback(is_fire);
//             } 
//         }   
//     } else if (msg->armors_num == ARMOR_NUM_OUTPOST) {
//         calculateArmorPosition(msg, false, true);
//         for (size_t i = 0; i < tmp_yaws.size(); i++) {
//             idx = selectArmor(msg, true);
//             is_fire = tmp_yaws[idx] >= min_yaw_in_cycle && tmp_yaws[idx] <= max_yaw_in_cycle;
//             if (fireCallback) {
//                 fireCallback(is_fire);
//             } 
//         }   
//     } else {
//         calculateArmorPosition(msg, false, false);
//         for (size_t i = 0; i < tmp_yaws.size(); i++) {
//             idx = selectArmor(msg, true);
//             is_fire = tmp_yaws[idx] >= min_yaw_in_cycle && tmp_yaws[idx] <= max_yaw_in_cycle;
//             if (fireCallback) {
//                 fireCallback(is_fire);
//             } 
//         }   
//     }

//     auto pitch_and_yaw = calculatePitchAndYaw(idx, msg, timeDelay, s_bias, z_bias, current_v, false,aim_x,aim_y,aim_z);
//     send_pitch = pitch_and_yaw.first;
//     send_yaw = pitch_and_yaw.second;
// }

/**
* @brief 根据最优决策得出被击打装甲板 自动解算弹道
* @param send_pitch:rad  传出pitch
* @param send_yaw:rad    传出yaw
* @param aim_x:传出aim_x  打击目标的x
* @param aim_y:传出aim_y  打击目标的y
* @param aim_z:传出aim_z  打击目标的z
*/
void SolveTrajectory::autoSolveTrajectory(float& send_pitch, float& send_yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
    // if(msg->v_yaw < 20.0f){
    //     fireLogicIsTop(send_pitch, send_yaw, aim_x, aim_y, aim_z, msg);
    // }
    // else{
    //     fireLogicOnlycentry(send_pitch, send_yaw, aim_x, aim_y, aim_z, msg);
    // }

    //* 优先开火逻辑
    fireLogicIsTop(send_pitch, send_yaw, aim_x, aim_y, aim_z, msg);
}

// 从坐标轴正向看向原点，逆时针方向为正

}  // namespace rm_auto_aim
