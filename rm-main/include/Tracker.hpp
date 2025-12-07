#ifndef TRACKER_HPP
#define TRACKER_HPP
#include "Kamal.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

class Tracker 
{
public:
    Tracker();

    const Eigen::Matrix<double, 6, 1>& operator()(const Eigen::Matrix<double, 3, 1>& measurement,double dt, double error = 1);

private:
    bool is_first_ = true;
    Kalman<6, 3> kalman_
    {
    (Eigen::Matrix<double, 3, 6>() << 
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0).finished()
    };

    Eigen::Matrix<double, 3, 1> G_R;//传感器噪声驱动矩阵
    Eigen::Matrix<double, 1, 1> r;//传感器噪声源协方差矩阵

    Eigen::Matrix<double, 3, 3> q;//过程噪声源协方差矩阵

    

};


#endif // TRACKER_HPP