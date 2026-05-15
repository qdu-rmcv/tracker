#include "armor_detector/pnp_solver.hpp"

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{
PnPSolver::PnPSolver(const std::array<double, 9>& camera_matrix,
                     const std::vector<double>& dist_coeffs,
                     PnpFilterParams filter_params)
    : camera_matrix_(cv::Mat(3, 3, CV_64F)),
      dist_coeffs_(cv::Mat::zeros(1, 5, CV_64F)),
      pnp_filter_params_(filter_params)
{
  std::memcpy(camera_matrix_.data, camera_matrix.data(), 9 * sizeof(double));
  size_t num_dist_coeffs = std::min(dist_coeffs.size(), static_cast<size_t>(5));
  std::memcpy(dist_coeffs_.data, dist_coeffs.data(), num_dist_coeffs * sizeof(double));
}

bool PnPSolver::SolvePnP(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec)
{
  std::array<cv::Point2f, 4> image_armor_points = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

  // Solve pnp
  auto object_points =
      armor.type == ArmorType::SMALL ? SMALL_ARMOR_POINTS : LARGE_ARMOR_POINTS;

  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> reprojection_errors;
  int solutions = cv::solvePnPGeneric(object_points, image_armor_points, camera_matrix_,
                                      dist_coeffs_, rvecs, tvecs, false,
                                      cv::SOLVEPNP_IPPE,  // 使用 IPPE 算法获取多个解
                                      cv::noArray(), cv::noArray(), reprojection_errors);

  if (pnp_filter_params_.new_pnp_filter_method)
  {
    if (solutions <= 0 || rvecs.empty() || tvecs.empty())
    {
      return false;
    }

    return SelectBestFilteredPnPResult(object_points, rvecs, tvecs, reprojection_errors,
                                       rvec, tvec);
  }
  else
  {
    if (solutions == 0)
    {
      return false;
    }

    double z_data[3]{0, 0, 10};
    cv::Mat z_vector(cv::Size(1, 3), CV_64FC1, z_data);

    cv::Mat r_0, r_1;
    cv::Rodrigues(rvecs.front(), r_0);
    cv::Rodrigues(rvecs.back(), r_1);

    cv::Mat z_camera_0 = r_0 * z_vector + tvecs.front();
    cv::Mat z_camera_1 = r_1 * z_vector + tvecs.back();

    if (z_camera_0.at<double>(2, 0) > 0)
    {
      rvec = rvecs.front();
      tvec = tvecs.front();
    }
    else
    {
      rvec = rvecs.back();
      tvec = tvecs.back();
    }
  }
  return true;
}

PnPSolver::Candidate PnPSolver::EvaluateCandidate(
    const cv::Mat& cand_rvec, const cv::Mat& cand_tvec, double reprojection_error,
    const std::array<cv::Point3f, 4>& object_points)
{
  Candidate c;
  c.rvec = cand_rvec.clone();
  c.tvec = cand_tvec.clone();
  c.reprojection_error = reprojection_error;

  double tz = cand_tvec.at<double>(2, 0);
  if (tz <= 0.0)
  {
    std::cerr << "tz <= 0! pnp solve faild!\n";
    return c;
  }

  cv::Mat r;
  cv::Rodrigues(cand_rvec, r);

  for (const auto& p : object_points)
  {
    cv::Mat x = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);
    cv::Mat xc = r * x + cand_tvec;
    if (xc.at<double>(2, 0) <= 1e-6)
    {
      std::cerr << "Xc point <= 0! pnp solve faild!\n";
      return c;
    }
  }

  cv::Mat normal_cam = r * (cv::Mat_<double>(3, 1) << 1.0, 0.0, 0.0);
  cv::Mat center_dir = cand_tvec / cv::norm(cand_tvec);

  double dot_nc = -(normal_cam.at<double>(0, 0) * center_dir.at<double>(0, 0) +
                    normal_cam.at<double>(1, 0) * center_dir.at<double>(1, 0) +
                    normal_cam.at<double>(2, 0) * center_dir.at<double>(2, 0));

  if (dot_nc >= pnp_filter_params_.max_normal_dot)
  {
    std::cerr << "dot_nc too large! pnp solve faild!\n"
              << "dot_nc: " << dot_nc << '\n';
    return c;
  }

  double normal_penalty = dot_nc + 1.0;

  c.score = pnp_filter_params_.reproj_weight * reprojection_error +
            pnp_filter_params_.normal_weight * normal_penalty;

  c.valid = true;
  return c;
}

bool PnPSolver::SelectBestFilteredPnPResult(
    const std::array<cv::Point3f, 4>& object_points, const std::vector<cv::Mat>& rvecs,
    const std::vector<cv::Mat>& tvecs, const std::vector<double>& reprojection_errors,
    cv::Mat& rvec, cv::Mat& tvec)
{
  Candidate best;

  for (size_t i = 0; i < rvecs.size(); ++i)
  {
    double err = (i < reprojection_errors.size()) ? reprojection_errors[i] : 1e9;
    Candidate cand = EvaluateCandidate(rvecs[i], tvecs[i], err, object_points);

    if (!cand.valid)
    {
      continue;
    }

    if (!best.valid || cand.score < best.score)
    {
      best = cand;
    }
  }

  if (!best.valid)
  {
    return false;
  }

  rvec = best.rvec;
  tvec = best.tvec;
  return true;
}

float PnPSolver::CalculateDistanceToCenter(
    const cv::Point2f& image_point)  // 计算给定图像点到图像中心的距离
{
  float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
  float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
  return static_cast<float>(cv::norm(image_point - cv::Point2f(cx, cy)));
}

void PnPSolver::SetCameraInfo(const std::array<double, 9>& camera_matrix,
                              const std::vector<double>& distortion_coefficients)
{
  std::memcpy(camera_matrix_.data, camera_matrix.data(), 9 * sizeof(double));
  size_t num_dist_coeffs =
      std::min(distortion_coefficients.size(), static_cast<size_t>(5));
  std::memcpy(dist_coeffs_.data, distortion_coefficients.data(),
              num_dist_coeffs * sizeof(double));
}

}  // namespace rm_auto_aim
