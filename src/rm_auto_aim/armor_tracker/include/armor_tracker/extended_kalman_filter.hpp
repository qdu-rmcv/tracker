#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <functional>

namespace rm_auto_aim
{

class ExtendedKalmanFilter
{
 public:
  ExtendedKalmanFilter() = default;

  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  explicit ExtendedKalmanFilter(const VecVecFunc& f, const VecVecFunc& h,
                                const VecMatFunc& j_f, const VecMatFunc& j_h,
                                const VoidMatFunc& u_q, const VecMatFunc& u_r,
                                const Eigen::MatrixXd& P0);

  // Set the initial state
  void SetState(const Eigen::VectorXd& x0);
  void SetState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

  // Get the estimated state
  Eigen::VectorXd GetState();

  // Compute a predicted state
  Eigen::MatrixXd Predict();

  // Update the estimated state based on measurement by IEKF
  Eigen::MatrixXd Update(const Eigen::VectorXd& z);

  // 新增：给 MatchArmor 用
  Eigen::VectorXd ComputeInnovation(const Eigen::VectorXd& z) const;
  Eigen::VectorXd ComputeInnovation(const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& x) const;
  double ComputeNIS(const Eigen::VectorXd& z) const;

  // Get the health rate
  double GetHealthRate();

 private:
  static constexpr int K_IEKF_ITERATIONS = 5;

  static double NormalizeAngle(double a) { return std::remainder(a, 2.0 * M_PI); }

  // Process nonlinear vector function
  VecVecFunc f;
  // Observation nonlinear vector function
  VecVecFunc h;
  // Jacobian of f()
  VecMatFunc jacobian_f;
  Eigen::MatrixXd F;
  // Jacobian of h()
  VecMatFunc jacobian_h;
  Eigen::MatrixXd H;
  // Process noise covariance matrix
  VoidMatFunc update_Q;
  Eigen::MatrixXd Q;
  // Measurement noise covariance matrix
  VecMatFunc update_R;
  Eigen::MatrixXd R;

  // Priori error estimate covariance matrix
  Eigen::MatrixXd P_pri;
  // Posteriori error estimate covariance matrix
  Eigen::MatrixXd P_post;

  // Kalman gain
  Eigen::MatrixXd K;

  // System dimensions
  int n;

  // N-size identity
  Eigen::MatrixXd I;

  // Priori state
  Eigen::VectorXd x_pri;
  // Posteriori state
  Eigen::VectorXd x_post;

  std::deque<double> nis_window_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_