#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78
#define fly_time 0.1f
typedef unsigned char uint8_t;

#include "buff_interfaces/msg/velocity.hpp"
#include "buff_interfaces/msg/rune.hpp"

#include <iostream>

namespace rm_buff
{
class BuffSolveTrajectory
{
public:

    BuffSolveTrajectory(const float &k, const int &bias_time, const float &s_bias, const float &z_bias);
    
    float k;             //弹道系数

    //自身参数
    //enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    float current_v;      //当前弹速

    //目标参数
    int bias_time;        //偏置时间
    float s_bias;         //枪口前推的距离
    float z_bias;         //yaw轴电机到枪口水平面的垂直距离


    void init(const buff_interfaces::msg::Velocity::SharedPtr velocity_msg);

    //单方向空气阻力模型
    float monoDirectionalAirResistanceModel(float s, float v, float angle);

    //pitch弹道补偿
    float pitchTrajectoryCompensation(float s, float y, float v);

    void calculateBuffPosition(const buff_interfaces::msg::Rune::SharedPtr msg,
                                float& target_x, float& target_y, float& target_z);

    std::pair<float, float> calculatePitchAndYaw(float timeDelay, float s_bias, float z_bias, float current_v,float& target_x, float& target_y, float& target_z,const buff_interfaces::msg::Rune::SharedPtr msg);

    //! 根据最优决策得出被击打装甲板 自动解算弹道
    void autoBuffSolveTrajectory(float& send_pitch, float& send_yaw,const buff_interfaces::msg::Rune::SharedPtr msg,
                                float& target_x, float& target_y, float& target_z);

private:    
};

} // namespace rm_buff

#endif /*__SOLVETRAJECTORY_H__*/
