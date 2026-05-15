#include "armor_tracker/extended_kalman_filter.hpp"

namespace rm_auto_aim
{
/*
f: 过程函数
h: 观测函数
j_f: 过程函数的雅可比矩阵
j_h: 测量函数的雅可比矩阵
u_q: 过程噪声协方差矩阵
u_r: 测量噪声协方差矩阵
P0: 初始状态协方差矩阵
*/
ExtendedKalmanFilter::ExtendedKalmanFilter(const VecVecFunc& f, const VecVecFunc& h,
                                           const VecMatFunc& j_f, const VecMatFunc& j_h,
                                           const VoidMatFunc& u_q, const VecMatFunc& u_r,
                                           const Eigen::MatrixXd& P0)
    : f(f),
      h(h),
      jacobian_f(j_f),
      jacobian_h(j_h),
      update_Q(u_q),
      update_R(u_r),
      P_post(P0),
      n(P0.rows()),
      I(Eigen::MatrixXd::Identity(n, n)),
      x_pri(n),
      x_post(n)
{
}

void ExtendedKalmanFilter::SetState(const Eigen::VectorXd& x0) { x_post = x0; }
void ExtendedKalmanFilter::SetState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
  x_post = x0;
  P_post = P0;
  nis_window_.clear();
}

Eigen::MatrixXd ExtendedKalmanFilter::Predict()
{
  F = jacobian_f(x_post);
  Q = update_Q();

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::VectorXd ExtendedKalmanFilter::ComputeInnovation(const Eigen::VectorXd& z) const
{
  return ComputeInnovation(z, x_pri);
}

Eigen::VectorXd ExtendedKalmanFilter::ComputeInnovation(const Eigen::VectorXd& z,
                                                        const Eigen::VectorXd& x) const
{
  Eigen::VectorXd innovation = z - h(x);
  if (innovation.size() > 3)
  {
    innovation(3) = NormalizeAngle(innovation(3));
  }
  return innovation;
}

double ExtendedKalmanFilter::ComputeNIS(const Eigen::VectorXd& z) const
{
  const Eigen::MatrixXd H_local = jacobian_h(x_pri);
  const Eigen::MatrixXd R_local = update_R(x_pri);
  const Eigen::VectorXd innovation = ComputeInnovation(z);
  const Eigen::MatrixXd S = H_local * P_pri * H_local.transpose() + R_local;

  return innovation.dot(S.ldlt().solve(innovation));
}

Eigen::MatrixXd ExtendedKalmanFilter::Update(const Eigen::VectorXd& z)
{
  const double nis = ComputeNIS(z);

  Eigen::VectorXd x_iter = x_pri;

  for (int i = 0; i < K_IEKF_ITERATIONS; ++i)
  {
    H = jacobian_h(x_iter);
    R = update_R(x_iter);

    const Eigen::MatrixXd S = H * P_pri * H.transpose() + R;
    const Eigen::MatrixXd S_inv =
        S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.rows()));
    K = P_pri * H.transpose() * S_inv;

    Eigen::VectorXd innovation = ComputeInnovation(z, x_iter) + H * (x_iter - x_pri);
    if (innovation.size() > 3)
    {
      innovation(3) = NormalizeAngle(innovation(3));
    }

    x_iter = x_pri + K * innovation;
  }

  x_post = x_iter;

  H = jacobian_h(x_post);
  R = update_R(x_post);
  const Eigen::MatrixXd S = H * P_pri * H.transpose() + R;
  const Eigen::MatrixXd S_inv =
      S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.rows()));
  K = P_pri * H.transpose() * S_inv;

  const Eigen::MatrixXd I_KH = I - K * H;
  P_post = I_KH * P_pri * I_KH.transpose() + K * R * K.transpose();

  nis_window_.push_back(nis);
  if (nis_window_.size() > 100)
  {
    nis_window_.pop_front();
  }

  return x_post;
}

Eigen::VectorXd ExtendedKalmanFilter::GetState() { return x_post; }

double ExtendedKalmanFilter::GetHealthRate()
{
  if (nis_window_.size() < 20)
  {
    return 1.0;
  }
  int health = static_cast<int>(std::count_if(nis_window_.begin(), nis_window_.end(),
                                              [](double v)
                                              { return v < 9.49; }));  // chi2(4, 0.05)
  return static_cast<double>(health) / nis_window_.size();
}

}  // namespace rm_auto_aim