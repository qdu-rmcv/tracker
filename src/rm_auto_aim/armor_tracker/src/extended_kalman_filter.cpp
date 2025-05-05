// Copyright (C) 2022 ChenJun

#include "armor_tracker/extended_kalman_filter.hpp"

namespace rm_auto_aim
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
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

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  // 使用当前状态和创新向量（预测误差）动态调整 Q
  F = jacobian_f(x_post);

  x_pri = f(x_post);

  Q = update_Q(x_pri - x_post);  // x_pri - x_post 是预测误差

  P_pri = F * P_post * F.transpose() + Q;

  // 如果没有测量更新，直接将预测值作为后验值
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  // 使用当前测量值和创新向量（测量误差）动态调整 R
  H = jacobian_h(x_pri);
  R = update_R(z);  // z - h(x_pri) 是创新向量

  // 计算卡尔曼增益
  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();

  // 更新状态和协方差矩阵
  x_post = x_pri + K * (z - h(x_pri));  // 更新状态
  P_post = (I - K * H) * P_pri;        // 更新协方差矩阵

  return x_post;
}

}  // namespace rm_auto_aim