#include "armor_detector/light_corner_corrector.hpp"

#include <cmath>
#include <numeric>
#include <opencv2/imgproc.hpp>

namespace rm_auto_aim
{

void LightCornerCorrector::CorrectCorners(std::vector<Armor> &armors,
                                          const cv::Mat &gray_img)
{
  // 如果灯条宽度过小，则不进行校正
  constexpr int PASS_OPTIMIZE_WIDTH = 3;

  for (auto &armor : armors)
  {
    if (armor.left_light.width > PASS_OPTIMIZE_WIDTH)
    {
      // 找到灯条的对称轴
      SymmetryAxis left_axis = FindSymmetryAxis(gray_img, armor.left_light);
      armor.left_light.center = left_axis.centroid;
      armor.left_light.axis = left_axis.direction;
      // 查找灯条的角点
      if (cv::Point2f t = FindCorner(gray_img, armor.left_light, left_axis, "top");
          t.x > 0)
      {
        armor.left_light.top = t;
      }
      if (cv::Point2f b = FindCorner(gray_img, armor.left_light, left_axis, "bottom");
          b.x > 0)
      {
        armor.left_light.bottom = b;
      }
    }

    if (armor.right_light.width > PASS_OPTIMIZE_WIDTH)
    {
      // 找到灯条的对称轴
      SymmetryAxis right_axis = FindSymmetryAxis(gray_img, armor.right_light);
      armor.right_light.center = right_axis.centroid;
      armor.right_light.axis = right_axis.direction;
      // 查找灯条的角点
      if (cv::Point2f t = FindCorner(gray_img, armor.right_light, right_axis, "top");
          t.x > 0)
      {
        armor.right_light.top = t;
      }
      if (cv::Point2f b = FindCorner(gray_img, armor.right_light, right_axis, "bottom");
          b.x > 0)
      {
        armor.right_light.bottom = b;
      }
      armor.left_light.length = cv::norm(armor.left_light.top - armor.left_light.bottom);
      armor.left_light.tilt_angle = static_cast<float>(
          std::atan2(armor.left_light.axis.y, armor.left_light.axis.x) * 180.0 / CV_PI);
      armor.right_light.length =
          cv::norm(armor.right_light.top - armor.right_light.bottom);
      armor.right_light.tilt_angle = static_cast<float>(
          std::atan2(armor.right_light.axis.y, armor.right_light.axis.x) * 180.0 / CV_PI);
    }
  }
}

SymmetryAxis LightCornerCorrector::FindSymmetryAxis(const cv::Mat &gray_img,
                                                    const Light &light)
{
  // 缩放
  cv::Rect light_box = light.boundingRect();
  light_box.x -= static_cast<int>(light_box.width * scale_);
  light_box.y -= static_cast<int>(light_box.height * scale_);
  light_box.width += static_cast<int>(light_box.width * scale_ * 2);
  light_box.height += static_cast<int>(light_box.height * scale_ * 2);

  // 检查并裁剪到图像边界
  light_box.x = std::max(light_box.x, 0);
  light_box.x = std::min(light_box.x, gray_img.cols - 1);
  light_box.y = std::max(light_box.y, 0);
  light_box.y = std::min(light_box.y, gray_img.rows - 1);
  light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
  light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

  // 获取并归一化灯条区域图像
  cv::Mat roi = gray_img(light_box);
  // 对roi，仅保留最大的轮廓
  // std::vector<std::vector<cv::Point>> contours;
  // cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  // if (!contours.empty())
  // {
  //   auto max_contour_it = std::max_element(
  //       contours.begin(), contours.end(),
  //       [](const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2)
  //       { return cv::contourArea(c1) < cv::contourArea(c2); });
  //   cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
  //   cv::fillPoly(mask, {*max_contour_it}, 255);
  //   cv::bitwise_and(roi, mask, roi);
  // }
  double mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, max_brightness_, cv::NORM_MINMAX);

  // 计算质心
  cv::Moments moments = cv::moments(roi, false);
  cv::Point2f centroid =
      cv::Point2f(static_cast<float>(moments.m10) / static_cast<float>(moments.m00),
                  static_cast<float>(moments.m01) / static_cast<float>(moments.m00)) +
      cv::Point2f(static_cast<float>(light_box.x), static_cast<float>(light_box.y));

  // 初始化点云
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; i++)
  {
    for (int j = 0; j < roi.cols; j++)
    {
      auto num_points = static_cast<int>(std::round(roi.at<float>(i, j)));
      for (int k = 0; k < num_points; k++)
      {
        points.emplace_back(cv::Point2f(static_cast<float>(j), static_cast<float>(i)));
      }
    }
  }
  cv::Mat points_mat = cv::Mat(points).reshape(1);

  // PCA
  auto pca = cv::PCA(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // 提取对称轴向量
  cv::Point2f axis =
      cv::Point2f(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));

  // 归一化轴向量
  axis = axis / cv::norm(axis);

  if (axis.y > 0)
  {
    axis = -axis;
  }

  return SymmetryAxis{.centroid = centroid, .direction = axis, .mean_val = mean_val};
}

cv::Point2f LightCornerCorrector::FindCorner(const cv::Mat &gray_img, const Light &light,
                                             const SymmetryAxis &axis,
                                             const std::string &order)
{
  auto in_image = [&gray_img](const cv::Point &point) -> bool
  {
    return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 &&
           point.y < gray_img.rows;
  };

  auto distance = [](double x0, double y0, double x1, double y1) -> double
  { return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1)); };

  double oper = order == "top" ? 1.0 : -1.0;
  double l = light.length;
  double dx = axis.direction.x * oper;
  double dy = axis.direction.y * oper;

  std::vector<cv::Point2f> candidates;

  // 选择多个角点候选并取平均作为最终角点
  int n = static_cast<int>(light.width - 2);
  int half_n = static_cast<int>(std::round(n / 2));
  for (int i = -half_n; i <= half_n; i++)
  {
    // double x0 = axis.centroid.x + l * START * dx + i;
    // double y0 = axis.centroid.y + l * START * dy;

    cv::Point2f normal(-axis.direction.y, axis.direction.x);
    cv::Point2f start = axis.centroid + l * start_ * axis.direction * oper + i * normal;
    float x0 = start.x;
    float y0 = start.y;

    if (!in_image(cv::Point(x0, y0)))
    {
      continue;
    }

    cv::Point2f prev = cv::Point2f(x0, y0);
    cv::Point2f corner = cv::Point2f(x0, y0);
    double max_brightness_diff = 0;
    bool has_corner = false;
    // 沿对称轴搜索具有最大亮度差的角点
    for (double x = x0 + dx, y = y0 + dy; distance(x, y, x0, y0) < l * (end_ - start_);
         x += dx, y += dy)
    {
      cv::Point2f cur = cv::Point2f(x, y);
      if (!in_image(cv::Point(cur)))
      {
        break;
      }

      double brightness_diff = gray_img.at<uchar>(prev) - gray_img.at<uchar>(cur);
      if (brightness_diff > max_brightness_diff &&
          gray_img.at<uchar>(prev) > axis.mean_val)
      {
        max_brightness_diff = brightness_diff;
        corner = prev;
        has_corner = true;
      }

      prev = cur;
    }

    if (has_corner)
    {
      candidates.emplace_back(corner);
    }
  }
  if (!candidates.empty())
  {
    cv::Point2f result =
        std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
    return result / static_cast<float>(candidates.size());
  }

  return cv::Point2f(-1, -1);
}

}  // namespace rm_auto_aim
