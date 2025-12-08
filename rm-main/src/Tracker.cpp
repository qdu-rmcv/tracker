#include "../include/Tracker.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

Tracker::Tracker() : is_first_(true) 
{
    //d（距离mm）协方差矩阵,构建观测噪声源矩阵
    this->R <<
        2500, 0,    0,
        0,    2500, 0,
        0,    0,    2500;

    //加速度mm/s^2噪声协方差,构建过程噪声源矩阵
    this->q<< 
        2500, 0,    0,
        0,    2500, 0,
        0,    0,    2500;

}


const Eigen::Matrix<double, 6, 1>& Tracker::operator()(const Eigen::Matrix<double, 3, 1>& measurement,double dt,double error)
{
    if(is_first_)
    {
        is_first_ = false;
        kalman_.StateVector().block<3,1>(0,0) = measurement;
        return kalman_.StateVector();
    }

    //1更新状态协方差矩阵
    auto& P = kalman_.Covariance_State();
    //1.1构建状态转移矩阵
    Eigen::Matrix<double, 6, 6> H;
    H << 
        1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    //1.2构建过程噪声协方差矩阵
    Eigen::Matrix<double, 6, 6> Q;
    //过程噪声驱动矩阵
    Eigen::Matrix<double, 6, 3> G_Q;
    G_Q <<
        0.5*dt*dt, 0, 0,
        0, 0.5*dt*dt, 0,
        0, 0, 0.5*dt*dt,
        dt, 0, 0,
        0, dt, 0,
        0, 0, dt;

    Q = G_Q * this->q * G_Q.transpose();

    P = H * P * H.transpose() + Q;

    //2构建测量协方差矩阵
    Eigen::Matrix<double, 3, 3>& R = kalman_.Covariance_Measurement();
    
    R = R * error;
    
    //更新状态向量为先验状态向量
    kalman_.StateVector() = H * kalman_.StateVector();

    return this->kalman_.update(measurement);;
}