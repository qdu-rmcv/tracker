#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

namespace rm_auto_aim
{
constexpr int RED = 0;
constexpr int BLUE = 1;

enum class ArmorType : uint8_t
{
  SMALL,
  LARGE,
  INVALID
};

constexpr std::array<std::string_view, 3> ARMOR_TYPE_STR = {"small", "large", "invalid"};
struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(const cv::RotatedRect& box)
      : cv::RotatedRect(box),
        tilt_angle(std::atan2(
            std::abs(top.x - bottom.x),
            std::abs(top.y -
                     bottom.y)))  // Light继承自cv::RotatedRect，可以直接使用其成员函数
  {
    cv::Point2f p[4];  // 灯条四个点
    box.points(p);     // rotatedrect的points函数可以获取四个点的坐标
    std::sort(p, p + 4,
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;  // 灯条顶部和底部中心点
    bottom = (p[2] + p[3]) / 2;

    // 计算灯条的长度，即顶部中心点和底部中心点之间的欧几里得距离
    length = cv::norm(top - bottom);
    // 计算灯条的宽度，即排序后前两个顶点之间的欧几里得距离
    width = cv::norm(p[0] - p[1]);

    axis = top - bottom;
    axis = axis / cv::norm(axis);

    // 在平面图像中，计算灯条相对于垂直方向的倾斜角度（弧度制）
    tilt_angle = static_cast<float>(tilt_angle / CV_PI * 180);
  }

  explicit Light(const cv::Point2f& top, const cv::Point2f& bottom, int color)
      : color(color),
        top(top),
        bottom(bottom),
        length(cv::norm(top - bottom)),
        width(0),
        tilt_angle(static_cast<float>(
            std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) / CV_PI *
            180.0))
  {
    center = (top + bottom) / 2.f;
    auto diff = top - bottom;
    auto norm = cv::norm(diff);
    axis = (norm > 1e-6) ? cv::Point2f(diff.x / static_cast<float>(norm),
                                       diff.y / static_cast<float>(norm))
                         : cv::Point2f(0, -1);
  }

  int color;
  cv::Point2f top;
  cv::Point2f bottom;
  cv::Point2f axis;
  double length;
  double width;
  float tilt_angle;
};

struct Armor
{
  Armor() = default;
  Armor(const Light& l1, const Light& l2)
  {
    if (l1.center.x < l2.center.x)
    {
      left_light = l1, right_light = l2;
    }
    else
    {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light;
  Light right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
