#include "armor_detector/armor_pose_optimizer.hpp"

namespace rm_auto_aim
{

ArmorPoseOptimizer::ArmorPoseOptimizer(const Params& params)
    : params_(params),
      ground_pitch_rad_(params.standard_pitch_deg * M_PI / 180.0),
      outpost_pitch_rad_(params.outpost_pitch_deg * M_PI / 180.0),
      half_range_rad_(params.range_search_half_range_deg * M_PI / 180.0),
      coarse_step_rad_(params.range_search_coarse_step_deg * M_PI / 180.0),
      fine_range_rad_(params.range_search_fine_range_deg * M_PI / 180.0),
      fine_step_rad_(params.range_search_fine_step_deg * M_PI / 180.0)
{
}

void ArmorPoseOptimizer::SetCameraIntrinsics(const cv::Mat& camera_matrix,
                                             const cv::Mat& dist_coeffs)
{
  cv_camera_matrix_ = camera_matrix.clone();
  cv_dist_coeffs_ = dist_coeffs.clone();

  // 提取内参
  fx_ = camera_matrix.at<double>(0, 0);
  fy_ = camera_matrix.at<double>(1, 1);
  cx_ = camera_matrix.at<double>(0, 2);
  cy_ = camera_matrix.at<double>(1, 2);

  intrinsics_set_ = true;
}

void ArmorPoseOptimizer::SetCameraToGimbalRotation(const Eigen::Matrix3d& R_gimbal_cam)
{
  r_gimbal_cam_ = R_gimbal_cam;
  r_cam_gimbal_ = R_gimbal_cam.transpose();
  transform_set_ = true;
}

std::array<Eigen::Vector3d, 4> ArmorPoseOptimizer::ExtractObjectPoints(ArmorType type)
{
  // 与 PnPSolver 中定义的顺序一致
  const double HY = (type == ArmorType::SMALL) ? SMALL_HALF_Y : LARGE_HALF_Y;
  const double HZ = (type == ArmorType::SMALL) ? SMALL_HALF_Z : LARGE_HALF_Z;
  return {Eigen::Vector3d(0, HY, -HZ), Eigen::Vector3d(0, HY, HZ),
          Eigen::Vector3d(0, -HY, HZ), Eigen::Vector3d(0, -HY, -HZ)};
}

Eigen::Matrix3d ArmorPoseOptimizer::BuildCameraRotation(double yaw,
                                                        double pitch_prior) const
{
  return r_cam_gimbal_ * Rz(yaw) * Ry(pitch_prior);
}

bool ArmorPoseOptimizer::ProjectPoints(const Eigen::Matrix3d& R_cam,
                                       const Eigen::Vector3d& t_cam,
                                       const std::array<Eigen::Vector3d, 4>& obj_points,
                                       std::array<Eigen::Vector2d, 4>& projected) const
{
  for (int i = 0; i < 4; ++i)
  {
    Eigen::Vector3d p_cam = R_cam * obj_points[i] + t_cam;
    double z = p_cam(2);

    if (z <= 1e-6)
    {
      return false;
    }

    double z_inv = 1.0 / z;
    projected[i](0) = fx_ * p_cam(0) * z_inv + cx_;
    projected[i](1) = fy_ * p_cam(1) * z_inv + cy_;
  }
  return true;
}

double ArmorPoseOptimizer::ComputeReprojectionError(
    double yaw, double pitch_prior, const Eigen::Vector3d& t_cam,
    const std::array<Eigen::Vector3d, 4>& obj_points,
    const std::array<Eigen::Vector2d, 4>& img_points_ud)
{
  Eigen::Matrix3d r_cam = BuildCameraRotation(yaw, pitch_prior);

  std::array<Eigen::Vector2d, 4> projected;
  if (!ProjectPoints(r_cam, t_cam, obj_points, projected))
  {
    return 1e9;
  }

  double total_error = 0.0;
  for (int i = 0; i < 4; ++i)
  {
    total_error += (img_points_ud[i] - projected[i]).squaredNorm();
  }
  return total_error;
}

bool ArmorPoseOptimizer::Optimize(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec)
{
  if (!intrinsics_set_ || !transform_set_)
  {
    return false;
  }

  // 检查 pitch、roll 先验约束
  Eigen::Matrix3d r_cam = RvecToRotationMatrix(rvec);
  double yaw_init = NAN, pitch_prior = NAN;
  if (!CheckConstraint(r_cam, yaw_init, pitch_prior))
  {
    return false;
  }

  // 图像点
  std::array<cv::Point2f, 4> image_points = {armor.left_light.bottom,
                                             armor.left_light.top, armor.right_light.top,
                                             armor.right_light.bottom};

  // 去畸变
  auto img_points_ud = UndistortPoints(image_points);

  // 物体点
  auto obj_points = ExtractObjectPoints(armor.type);

  // yaw 优化
  double yaw = yaw_init;
  Eigen::Vector3d t_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  bool opt_success = false;
  if (params_.optimize_method == Params::OptimizeMethod::RANGE_LM)
  {
    // 先范围搜索后 LM 微调
    double range_error = 0.0;
    opt_success =
        RunRangeSolve(yaw, pitch_prior, t_cam, obj_points, img_points_ud, &range_error);
    if (opt_success)
    {
      double yaw_lm = yaw;
      Eigen::Vector3d t_lm = t_cam;
      bool lm_success = RunLM(yaw_lm, pitch_prior, t_lm, obj_points, img_points_ud);

      if (lm_success)
      {
        double lm_error = ComputeReprojectionError(yaw_lm, pitch_prior, t_lm, obj_points,
                                                   img_points_ud);
        constexpr double K_ABS = 1e-6;
        constexpr double K_REL = 1e-4;  // 0.01%
        double accept_tol = std::max(K_ABS, K_REL * std::max(1.0, range_error));
        if (std::isfinite(lm_error) && lm_error < range_error - accept_tol)
        {
          yaw = yaw_lm;
          t_cam = t_lm;
        }
      }
    }
  }
  else if (params_.optimize_method == Params::OptimizeMethod::RANGE)
  {
    opt_success = RunRangeSolve(yaw, pitch_prior, t_cam, obj_points, img_points_ud);
  }
  else
  {
    // Levenberg-Marquardt 迭代：以 PnP 初始解为起点联合优化 (yaw, t)
    opt_success = RunLM(yaw, pitch_prior, t_cam, obj_points, img_points_ud);
  }

  if (!opt_success)
  {
    return false;
  }

  Eigen::Matrix3d r_cam_optimized = BuildCameraRotation(yaw, pitch_prior);
  rvec = RotationMatrixToRvec(r_cam_optimized);

  tvec.at<double>(0) = t_cam(0);
  tvec.at<double>(1) = t_cam(1);
  tvec.at<double>(2) = t_cam(2);

  return true;
}

bool ArmorPoseOptimizer::CheckConstraint(const Eigen::Matrix3d& R_cam, double& yaw_init,
                                         double& pitch_prior)
{
  // 转换到惯性系
  Eigen::Matrix3d r_gimbal = r_gimbal_cam_ * R_cam;

  Eigen::Vector3d euler_ypr = DecomposeZYX(r_gimbal);

  double measured_pitch = euler_ypr(1);  // 弧度
  double measured_roll = euler_ypr(2);

  // roll 必须接近 0°
  double roll_deg = std::abs(measured_roll) * 180.0 / M_PI;
  if (roll_deg > params_.max_roll_deviation)
  {
    return false;
  }

  if (is_outpost_)
  {
    pitch_prior = outpost_pitch_rad_;
  }
  else
  {
    pitch_prior = ground_pitch_rad_;
  }

  yaw_init = euler_ypr(0);
  return true;
}

std::array<Eigen::Vector2d, 4> ArmorPoseOptimizer::UndistortPoints(
    const std::array<cv::Point2f, 4>& points)
{
  std::vector<cv::Point2f> distorted(points.begin(), points.end());
  std::vector<cv::Point2f> undistorted;

  cv::undistortPoints(distorted, undistorted, cv_camera_matrix_, cv_dist_coeffs_,
                      cv::noArray(), cv_camera_matrix_);

  std::array<Eigen::Vector2d, 4> result;
  for (int i = 0; i < 4; ++i)
  {
    result[i] = Eigen::Vector2d(undistorted[i].x, undistorted[i].y);
  }
  return result;
}

bool ArmorPoseOptimizer::ComputeResidualAndJacobian(
    double yaw, double pitch_prior, const Eigen::Vector3d& t_cam,
    const std::array<Eigen::Vector3d, 4>& obj_points,
    const std::array<Eigen::Vector2d, 4>& img_points_ud,
    Eigen::Matrix<double, 8, 1>& residual, Eigen::Matrix<double, 8, 4>& jacobian)
{
  // 预计算
  Eigen::Matrix3d ry_prior = Ry(pitch_prior);
  Eigen::Matrix3d r_cam = BuildCameraRotation(yaw, pitch_prior);
  Eigen::Matrix3d d_r_cam_dyaw = r_cam_gimbal_ * DRzDyaw(yaw) * ry_prior;

  for (auto i = 0; i < 4; ++i)
  {
    // 变换到相机坐标系
    Eigen::Vector3d p_cam = r_cam * obj_points[i] + t_cam;
    double x = p_cam(0), y = p_cam(1), z = p_cam(2);

    // 点在相机后方或过近
    if (z <= 1e-6)
    {
      return false;
    }

    double z_inv = 1.0 / z;
    double z_inv2 = z_inv * z_inv;

    // 针孔投影
    double u_proj = fx_ * x * z_inv + cx_;
    double v_proj = fy_ * y * z_inv + cy_;

    // 观测 - 预测
    residual(2 * i + 0) = img_points_ud[i](0) - u_proj;
    residual(2 * i + 1) = img_points_ud[i](1) - v_proj;

    // 投影函数对 P_cam 的雅可比
    Eigen::Matrix<double, 2, 3> j_proj;
    j_proj << fx_ * z_inv, 0.0, -fx_ * x * z_inv2, 0.0, fy_ * z_inv, -fy_ * y * z_inv2;

    // P_cam 对 yaw 的偏导
    Eigen::Vector3d d_p_dyaw = d_r_cam_dyaw * obj_points[i];

    // 残差对参数 [yaw, tx, ty, tz] 的雅可比
    Eigen::Matrix<double, 2, 4> j_i;
    j_i.col(0) = -j_proj * d_p_dyaw;
    j_i.block<2, 3>(0, 1) = -j_proj;  // dP/dt = I

    int64_t index = static_cast<int64_t>(2) * i;
    jacobian.block<2, 4>(index, 0) = j_i;
  }

  return true;
}

bool ArmorPoseOptimizer::RunLM(double& yaw, double pitch_prior, Eigen::Vector3d& t_cam,
                               const std::array<Eigen::Vector3d, 4>& obj_points,
                               const std::array<Eigen::Vector2d, 4>& img_points_ud)
{
  double lambda = params_.initial_lambda;

  Eigen::Matrix<double, 8, 1> residual;
  Eigen::Matrix<double, 8, 4> jacobian;

  // 有点在相机后方
  if (!ComputeResidualAndJacobian(yaw, pitch_prior, t_cam, obj_points, img_points_ud,
                                  residual, jacobian))
  {
    return false;
  }

  double cost = residual.squaredNorm();
  bool ever_succeeded = false;  // 至少一次成功

  for (int iter = 0; iter < params_.max_iterations; ++iter)
  {
    // 正规方程
    Eigen::Matrix4d h = jacobian.transpose() * jacobian;

    Eigen::Vector4d g = jacobian.transpose() * residual;

    // 加阻尼
    Eigen::Matrix4d h_damped = h;
    for (int k = 0; k < 4; ++k)
    {
      h_damped(k, k) += lambda * std::max(h(k, k), 1e-10);
    }

    // 求解 H_damped * delta = -g
    // 这里 -g 对应让代价函数下降的方向。
    Eigen::Vector4d delta = h_damped.ldlt().solve(-g);

    double yaw_new = yaw + delta(0);
    Eigen::Vector3d t_cam_new = t_cam + delta.tail<3>();

    // 计算新的残差和代价
    Eigen::Matrix<double, 8, 1> residual_new;
    Eigen::Matrix<double, 8, 4> jacobian_new;

    // 检查新参数是否导致点落到相机后方
    if (!ComputeResidualAndJacobian(yaw_new, pitch_prior, t_cam_new, obj_points,
                                    img_points_ud, residual_new, jacobian_new))
    {
      lambda *= params_.lambda_scale_up;
      if (lambda > 1e10)
      {
        return ever_succeeded;
      }
      continue;
    }

    double cost_new = residual_new.squaredNorm();

    if (cost_new < cost)
    {
      // 接受更新，减小阻尼
      yaw = yaw_new;
      t_cam = t_cam_new;
      residual = residual_new;
      jacobian = jacobian_new;
      ever_succeeded = true;

      // 检查收敛条件
      double relative_cost_change = (cost - cost_new) / (cost + 1e-15);
      cost = cost_new;
      lambda /= params_.lambda_scale_down;
      lambda = std::max(lambda, 1e-15);

      if (delta.norm() < params_.convergence_eps ||
          relative_cost_change < params_.cost_convergence_eps)
      {
        return true;  // 收敛
      }
    }
    else
    {
      // 增大阻尼，不更新参数
      lambda *= params_.lambda_scale_up;

      // 阻尼过大说明已在极值点附近或问题有困难
      if (lambda > 1e10)
      {
        // 若从未成功过，说明初始点附近就无法改善，返回失败
        return ever_succeeded;
      }
    }
  }

  // 达到最大迭代次数
  // 若有过成功步骤，参数已被改善，视为成功；否则视为失败
  return ever_succeeded;
}

bool ArmorPoseOptimizer::SearchYaw(double& yaw, double pitch_prior,
                                   Eigen::Vector3d& t_cam,
                                   const std::array<Eigen::Vector3d, 4>& obj_points,
                                   const std::array<Eigen::Vector2d, 4>& img_points_ud,
                                   double* best_error_out)
{
  const double CENTER_YAW = yaw;
  double best_yaw = yaw;
  const Eigen::Vector3d FIXED_T = t_cam;
  double min_error =
      ComputeReprojectionError(yaw, pitch_prior, FIXED_T, obj_points, img_points_ud);

  for (double candidate = CENTER_YAW - half_range_rad_;
       candidate <= CENTER_YAW + half_range_rad_; candidate += coarse_step_rad_)
  {
    double error = ComputeReprojectionError(candidate, pitch_prior, FIXED_T, obj_points,
                                            img_points_ud);
    if (error < min_error)
    {
      min_error = error;
      best_yaw = candidate;
    }
  }

  // 搜索全部无效，放弃
  if (min_error >= 1e9)
  {
    return false;
  }

  // 精搜索
  const double FINE_START = best_yaw - fine_range_rad_;
  const double FINE_END = best_yaw + fine_range_rad_;
  for (double candidate = FINE_START; candidate <= FINE_END; candidate += fine_step_rad_)
  {
    double error = ComputeReprojectionError(candidate, pitch_prior, FIXED_T, obj_points,
                                            img_points_ud);
    if (error < min_error)
    {
      min_error = error;
      best_yaw = candidate;
    }
  }

  yaw = best_yaw;

  t_cam = FIXED_T;
  if (best_error_out)
  {
    *best_error_out = min_error;
  }
  return true;
}

bool ArmorPoseOptimizer::SearchYawWithT(
    double& yaw, double pitch_prior, Eigen::Vector3d& t_cam,
    const std::array<Eigen::Vector3d, 4>& obj_points,
    const std::array<Eigen::Vector2d, 4>& img_points_ud, double* best_error_out)
{
  const double CENTER_YAW = yaw;
  double best_yaw = yaw;
  Eigen::Vector3d best_t = t_cam;
  double min_error = 1e9;

  // 对每个候选 yaw 解析求解最优 t，消除 yaw-t 耦合
  for (double candidate = CENTER_YAW - half_range_rad_;
       candidate <= CENTER_YAW + half_range_rad_; candidate += coarse_step_rad_)
  {
    Eigen::Vector3d candidate_t;
    double error = EvaluateCandidateYaw(candidate, pitch_prior, obj_points, img_points_ud,
                                        candidate_t);
    if (error < min_error)
    {
      min_error = error;
      best_yaw = candidate;
      best_t = candidate_t;
    }
  }

  // 搜索全部无效，放弃
  if (min_error >= 1e9)
  {
    return false;
  }

  const double FINE_START = best_yaw - fine_range_rad_;
  const double FINE_END = best_yaw + fine_range_rad_;
  for (double candidate = FINE_START; candidate <= FINE_END; candidate += fine_step_rad_)
  {
    Eigen::Vector3d candidate_t;
    double error = EvaluateCandidateYaw(candidate, pitch_prior, obj_points, img_points_ud,
                                        candidate_t);
    if (error < min_error)
    {
      min_error = error;
      best_yaw = candidate;
      best_t = candidate_t;
    }
  }

  yaw = best_yaw;
  t_cam = best_t;
  if (best_error_out)
  {
    *best_error_out = min_error;
  }
  return true;
}

bool ArmorPoseOptimizer::RunRangeSolve(
    double& yaw, double pitch_prior, Eigen::Vector3d& t_cam,
    const std::array<Eigen::Vector3d, 4>& obj_points,
    const std::array<Eigen::Vector2d, 4>& img_points_ud, double* best_error_out)
{
  if (params_.range_fix_t_cam_)
  {
    return SearchYawWithT(yaw, pitch_prior, t_cam, obj_points, img_points_ud,
                          best_error_out);
  }
  else
  {
    return SearchYaw(yaw, pitch_prior, t_cam, obj_points, img_points_ud, best_error_out);
  }
  return true;
}

bool ArmorPoseOptimizer::SolveTranslationForFixedRotation(
    const std::array<Eigen::Vector3d, 4>& rotated_points,
    const std::array<Eigen::Vector2d, 4>& img_points_ud, Eigen::Vector3d& t_out) const
{
  // 构建线性系统 A * t = b
  Eigen::Matrix<double, 8, 3> a;
  Eigen::Matrix<double, 8, 1> b;

  for (int i = 0; i < 4; ++i)
  {
    const Eigen::Vector3d& q = rotated_points[i];

    double du = img_points_ud[i](0) - cx_;
    double dv = img_points_ud[i](1) - cy_;

    a(2L * i, 0) = fx_;
    a(2L * i, 1) = 0.0;
    a(2L * i, 2) = -du;
    b(2L * i) = du * q(2) - fx_ * q(0);

    a(2L * i + 1, 0) = 0.0;
    a(2L * i + 1, 1) = fy_;
    a(2L * i + 1, 2) = -dv;
    b(2L * i + 1) = dv * q(2) - fy_ * q(1);
  }

  // 正规方程求解 (A^T A) t = A^T b
  Eigen::Matrix3d at_a = a.transpose() * a;
  Eigen::Vector3d at_b = a.transpose() * b;

  Eigen::LDLT<Eigen::Matrix3d> ldlt(at_a);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive())
  {
    // 退化
    return false;
  }

  t_out = ldlt.solve(at_b);
  return true;
}

double ArmorPoseOptimizer::EvaluateCandidateYaw(
    double yaw, double pitch_prior, const std::array<Eigen::Vector3d, 4>& obj_points,
    const std::array<Eigen::Vector2d, 4>& img_points_ud, Eigen::Vector3d& t_out)
{
  Eigen::Matrix3d r_cam = BuildCameraRotation(yaw, pitch_prior);

  std::array<Eigen::Vector3d, 4> q;
  for (int i = 0; i < 4; ++i)
  {
    q[i] = r_cam * obj_points[i];
  }

  if (!SolveTranslationForFixedRotation(q, img_points_ud, t_out))
  {
    return 1e9;
  }

  double total_error = 0.0;
  for (int i = 0; i < 4; ++i)
  {
    Eigen::Vector3d p = q[i] + t_out;
    if (p(2) <= 1e-6)
    {
      return 1e9;
    }
    double z_inv = 1.0 / p(2);
    double du = img_points_ud[i](0) - (fx_ * p(0) * z_inv + cx_);
    double dv = img_points_ud[i](1) - (fy_ * p(1) * z_inv + cy_);
    total_error += du * du + dv * dv;
  }
  return total_error;
}

Eigen::Matrix3d ArmorPoseOptimizer::RvecToRotationMatrix(const cv::Mat& rvec)
{
  cv::Mat r_cv;
  cv::Rodrigues(rvec, r_cv);
  Eigen::Matrix3d r;
  cv::cv2eigen(r_cv, r);
  return r;
}

cv::Mat ArmorPoseOptimizer::RotationMatrixToRvec(const Eigen::Matrix3d& R)
{
  cv::Mat r_cv;
  cv::eigen2cv(R, r_cv);
  cv::Mat rvec;
  cv::Rodrigues(r_cv, rvec);
  return rvec;
}

Eigen::Vector3d ArmorPoseOptimizer::DecomposeZYX(const Eigen::Matrix3d& R)
{
  double pitch = std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
  double cos_pitch = std::cos(pitch);

  double yaw = NAN, roll = NAN;
  if (std::abs(cos_pitch) > 1e-6)
  {
    // 正常情况
    yaw = std::atan2(R(1, 0), R(0, 0));
    roll = std::atan2(R(2, 1), R(2, 2));
  }
  else
  {
    // 万向节锁，设 roll = 0
    roll = 0.0;
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }

  return Eigen::Vector3d(yaw, pitch, roll);
}

Eigen::Matrix3d ArmorPoseOptimizer::Rz(double yaw)
{
  double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::Matrix3d r;
  r << c, -s, 0, s, c, 0, 0, 0, 1;
  return r;
}

Eigen::Matrix3d ArmorPoseOptimizer::DRzDyaw(double yaw)
{
  double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::Matrix3d d_r;
  d_r << -s, -c, 0, c, -s, 0, 0, 0, 0;
  return d_r;
}

Eigen::Matrix3d ArmorPoseOptimizer::Ry(double pitch)
{
  double c = std::cos(pitch), s = std::sin(pitch);
  Eigen::Matrix3d r;
  r << c, 0, s, 0, 1, 0, -s, 0, c;
  return r;
}

}  // namespace rm_auto_aim