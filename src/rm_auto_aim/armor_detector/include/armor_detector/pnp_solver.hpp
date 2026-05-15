#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class PnPSolver
{
 public:
  struct PnpFilterParams
  {
    bool new_pnp_filter_method;
    double max_normal_dot;  // 更严格可设为 -0.1 / -0.2, 更宽松可设为 0.1 / 0.2
    // 评分权重
    double reproj_weight;
    double normal_weight;
  };

  PnPSolver(const std::array<double, 9>& camera_matrix,
            const std::vector<double>& distortion_coefficients,
            PnpFilterParams filter_params);

  // Get 3d position
  bool SolvePnP(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec);

  // Calculate the distance between armor center and image center
  float CalculateDistanceToCenter(const cv::Point2f& image_point);

  void SetCameraInfo(const std::array<double, 9>& camera_matrix,
                     const std::vector<double>& distortion_coefficients);

 private:
  struct Candidate
  {
    cv::Mat rvec;
    cv::Mat tvec;
    double reprojection_error = 1e9;
    double score = 1e18;
    bool valid = false;
  };

  bool SelectBestFilteredPnPResult(const std::array<cv::Point3f, 4>& object_points,
                                   const std::vector<cv::Mat>& rvecs,
                                   const std::vector<cv::Mat>& tvecs,
                                   const std::vector<double>& reprojection_errors,
                                   cv::Mat& rvec, cv::Mat& tvec);

  Candidate EvaluateCandidate(const cv::Mat& cand_rvec, const cv::Mat& cand_tvec,
                              double reprojection_error,
                              const std::array<cv::Point3f, 4>& object_points);

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  PnpFilterParams pnp_filter_params_;

  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_HEIGHT = 55;
  static constexpr float LARGE_ARMOR_WIDTH = 230;
  static constexpr float LARGE_ARMOR_HEIGHT = 55;

  // Four vertices of armor in 3d
  // Unit: m
  static constexpr double SMALL_HALF_Y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  static constexpr double SMALL_HALF_Z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  static constexpr double LARGE_HALF_Y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  static constexpr double LARGE_HALF_Z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  static const std::array<cv::Point3f, 4> SMALL_ARMOR_POINTS;
  static const std::array<cv::Point3f, 4> LARGE_ARMOR_POINTS;
};

inline const std::array<cv::Point3f, 4> PnPSolver::SMALL_ARMOR_POINTS = {{
    {0, SMALL_HALF_Y, -SMALL_HALF_Z},
    {0, SMALL_HALF_Y, SMALL_HALF_Z},
    {0, -SMALL_HALF_Y, SMALL_HALF_Z},
    {0, -SMALL_HALF_Y, -SMALL_HALF_Z},
}};

inline const std::array<cv::Point3f, 4> PnPSolver::LARGE_ARMOR_POINTS = {{
    {0, LARGE_HALF_Y, -LARGE_HALF_Z},
    {0, LARGE_HALF_Y, LARGE_HALF_Z},
    {0, -LARGE_HALF_Y, LARGE_HALF_Z},
    {0, -LARGE_HALF_Y, -LARGE_HALF_Z},
}};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
