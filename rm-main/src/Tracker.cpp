#include "../include/Tracker.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

Tracker::Tracker() : is_first_(true) 
{
    //d（距离cm）协方差矩阵,构建观测噪声源矩阵
    this->r << 25;

    //加速度cm/s^2噪声协方差,构建过程噪声源矩阵
    this->q<< 250000, 0, 0,
              0, 250000, 0,
              0, 0, 250000;

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
    
    //测量噪声驱动矩阵
    Eigen::Matrix<double, 3, 1> n = measurement/measurement.norm();
    this->G_R<<
        n(0)*n(0),
        n(1)*n(1),
        n(2)*n(2);
    R = this->G_R * this->r * this->G_R.transpose() * error;

    return this->kalman_.update(measurement);;
}