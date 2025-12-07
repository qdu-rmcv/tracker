#ifndef KALMAN_HPP
#define KALMAN_HPP
#include "eigen3/Eigen/Dense"

template<int x_dim, int z_dim, typename T = double >
class Kalman {
public:

    Kalman(const Eigen::Matrix<T, z_dim, x_dim>& H, 
           const Eigen::Matrix<T, x_dim, x_dim>& P_0 = Eigen::Matrix<T, x_dim, x_dim>::Identity(),
           const Eigen::Matrix<T, z_dim, z_dim>& R_0 = Eigen::Matrix<T, z_dim, z_dim>::Identity());

    const Eigen::Matrix<T, x_dim, 1>& update(const Eigen::Matrix<T, z_dim, 1>& Z);

    Eigen::Matrix<T, x_dim, 1>& StateVector();

    Eigen::Matrix<T, x_dim, x_dim>& Covariance_State();
    Eigen::Matrix<T, z_dim, z_dim>& Covariance_Measurement();

private:
    Eigen::Matrix<T, x_dim, 1> X_; // State vector

    Eigen::Matrix<T, x_dim, x_dim> P_; // Covariance matrix
    Eigen::Matrix<T, z_dim, z_dim> R_; // Covariance matrix
    const Eigen::Matrix<T, z_dim, x_dim> H_; // Measurement matrix
};

template<int x_dim, int z_dim, typename T>
Kalman<x_dim, z_dim, T>::Kalman(const Eigen::Matrix<T, z_dim, x_dim>& H, 
                                const Eigen::Matrix<T, x_dim, x_dim>& P_0,
                                const Eigen::Matrix<T, z_dim, z_dim>& R_0)
                                : H_(H), P_(P_0), R_(R_0), 
                                  X_(Eigen::Matrix<T, x_dim, 1>::Zero()){}

template<int x_dim, int z_dim, typename T>
Eigen::Matrix<T, x_dim, 1>& Kalman<x_dim, z_dim, T>::StateVector() { return X_; }


template<int x_dim, int z_dim, typename T>
Eigen::Matrix<T, x_dim, x_dim>& Kalman<x_dim, z_dim, T>::Covariance_State() { return P_; }

template<int x_dim, int z_dim, typename T>
Eigen::Matrix<T, z_dim, z_dim>& Kalman<x_dim, z_dim, T>::Covariance_Measurement() { return R_; }    

template<int x_dim, int z_dim, typename T>
const Eigen::Matrix<T, x_dim, 1>& Kalman<x_dim, z_dim, T>::update(const Eigen::Matrix<T, z_dim, 1>& Z) {
    // Kalman Gain
    Eigen::Matrix<T, x_dim, z_dim> K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse(); 
    // Update state estimate
    this->X_ = this->X_ + K * (Z - this->H_ * this->X_);
    // Update covariance estimate
    this->P_ = (Eigen::Matrix<T, x_dim, x_dim>::Identity() - K * this->H_) * this->P_ * (Eigen::Matrix<T, x_dim, x_dim>::Identity() - K * this->H_).transpose() + K * this->R_ * K.transpose();
    return this->X_;
}   


#endif // KALMAN_HPP