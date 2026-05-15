#include "armor_detector/detector.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace rm_auto_aim
{
namespace
{

template <typename T>
T get_parameter(rclcpp::Node& node, const std::string& name, const T& default_value)
{
  if (!node.has_parameter(name))
  {
    return node.declare_parameter<T>(name, default_value);
  }
  return node.get_parameter(name).get_value<T>();
}

}  // namespace

std::unique_ptr<Detector> Detector::Create(rclcpp::Node& node)
{
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");

  DetectorParams detector_params = {
      .binary_lower_thres = get_parameter<int>(node, "binary_lower_thres", 160),
      .binary_upper_thres = get_parameter<int>(node, "binary_upper_thres", 255),
      .detect_color = get_parameter<int>(node, "detect_color", RED),
      .l = {.min_ratio = get_parameter<double>(node, "light.min_ratio", 0.1),
            .max_ratio = get_parameter<double>(node, "light.max_ratio", 0.4),
            .max_angle = get_parameter<double>(node, "light.max_angle", 40.0)},
      .a = {.min_light_ratio = get_parameter<double>(node, "armor.min_light_ratio", 0.7),
            .min_small_center_distance =
                get_parameter<double>(node, "armor.min_small_center_distance", 0.8),
            .max_small_center_distance =
                get_parameter<double>(node, "armor.max_small_center_distance", 3.2),
            .min_large_center_distance =
                get_parameter<double>(node, "armor.min_large_center_distance", 3.2),
            .max_large_center_distance =
                get_parameter<double>(node, "armor.max_large_center_distance", 5.5),
            .max_angle = get_parameter<double>(node, "armor.max_angle", 35.0)},
      .c = {.model_path = pkg_path + "/model/mlp.onnx",
            .label_path = pkg_path + "/model/label.txt",
            .threshold = get_parameter<double>(node, "classifier_threshold", 0.7),
            .ignore_classes = get_parameter<std::vector<std::string>>(
                node, "ignore_classes", {"negative"})},
      .cc = {.use_corner_corrector = get_parameter<bool>(
                 node, "corner_corrector.use_corner_corrector", false),
             .max_brightness =
                 get_parameter<double>(node, "corner_corrector.max_brightness", 25.0),
             .scale = get_parameter<double>(node, "corner_corrector.scale", 0.07),
             .start = get_parameter<double>(node, "corner_corrector.start", 0.4),
             .end = get_parameter<double>(node, "corner_corrector.end", 0.6)}};

  return std::make_unique<Detector>(detector_params);
}

Detector::Detector(DetectorParams& params) : params_(params)
{
  classifier_ =
      std::make_unique<NumberClassifier>(params.c.model_path, params.c.label_path,
                                         params.c.threshold, params.c.ignore_classes);
  if (params_.cc.use_corner_corrector)
  {
    light_corner_corrector_ = std::make_unique<LightCornerCorrector>(
        params_.cc.max_brightness, params_.cc.scale, params_.cc.start, params_.cc.end);
    RCLCPP_ERROR(rclcpp::get_logger("armor_detector"),
                 "Using LightCornerCorrector with max_brightness: %f, scale: %f, start: "
                 "%f, end: %f",
                 params_.cc.max_brightness, params_.cc.scale, params_.cc.start,
                 params_.cc.end);
  }
}

DetectionResult Detector::Detect(const cv::Mat& input)
{
  debug_latencies_.clear();
  DetectionResult result;
  auto preprocess_start_time = std::chrono::steady_clock::now();
  binary_img_ = PreprocessImage(input);
  auto preprocess_end_time = std::chrono::steady_clock::now();
  auto preprocess_latency = std::chrono::duration_cast<std::chrono::microseconds>(
                                preprocess_end_time - preprocess_start_time)
                                .count();
  debug_latencies_.emplace_back("Preprocess", static_cast<uint64_t>(preprocess_latency));
  auto find_light_start_time = std::chrono::steady_clock::now();
  lights_ = FindLights(input, binary_img_);
  auto find_light_end_time = std::chrono::steady_clock::now();
  auto find_light_latency = std::chrono::duration_cast<std::chrono::microseconds>(
                                find_light_end_time - find_light_start_time)
                                .count();
  debug_latencies_.emplace_back("Find Lights", static_cast<uint64_t>(find_light_latency));
  auto match_light_start_time = std::chrono::steady_clock::now();
  armors_ = MatchLights(lights_);
  auto match_light_end_time = std::chrono::steady_clock::now();
  auto match_light_latency = std::chrono::duration_cast<std::chrono::microseconds>(
                                 match_light_end_time - match_light_start_time)
                                 .count();
  debug_latencies_.emplace_back("Match Lights",
                                static_cast<uint64_t>(match_light_latency));

  if (!armors_.empty())
  {
    if (light_corner_corrector_)
    {
      auto corner_correct_start_time = std::chrono::steady_clock::now();

      light_corner_corrector_->CorrectCorners(armors_, gray_img_);

      auto corner_correct_end_time = std::chrono::steady_clock::now();
      auto corner_correct_latency =
          std::chrono::duration_cast<std::chrono::microseconds>(corner_correct_end_time -
                                                                corner_correct_start_time)
              .count();
      debug_latencies_.emplace_back("Corner Correct",
                                    static_cast<uint64_t>(corner_correct_latency));
    }
    auto classify_start_time = std::chrono::steady_clock::now();
    classifier_->ExtractNumbers(input, armors_);
    classifier_->Classify(armors_);
    auto classify_end_time = std::chrono::steady_clock::now();
    auto classify_latency = std::chrono::duration_cast<std::chrono::microseconds>(
                                classify_end_time - classify_start_time)
                                .count();
    debug_latencies_.emplace_back("Classify", static_cast<uint64_t>(classify_latency));
  }

  auto detect_result_fill_start_time = std::chrono::steady_clock::now();
  result.armors = armors_;
  result.binary_image = binary_img_;
  result.debug_data = GetDebugData();

  const auto& numbers_image = GetNumbersImage();
  if (!numbers_image.empty())
  {
    result.numbers_image = numbers_image;
  }
  auto detect_result_fill_end_time = std::chrono::steady_clock::now();
  auto detect_result_fill_latency =
      std::chrono::duration_cast<std::chrono::microseconds>(detect_result_fill_end_time -
                                                            detect_result_fill_start_time)
          .count();
  debug_latencies_.emplace_back("Fill Result",
                                static_cast<uint64_t>(detect_result_fill_latency));

  return result;
}

cv::Mat Detector::PreprocessImage(const cv::Mat& rgb_img)  // 图像预处理
{
  cv::cvtColor(rgb_img, gray_img_, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  cv::inRange(gray_img_, cv::Scalar(params_.binary_lower_thres),
              cv::Scalar(params_.binary_upper_thres), binary_img);
  return binary_img;
}

std::vector<Light> Detector::FindLights(const cv::Mat& rbg_img,
                                        const cv::Mat& binary_img) noexcept
{
  using std::vector;
  vector<vector<cv::Point>> contours;  // 定义一个向量，用于存储图像中检测到的所有轮廓
  vector<cv::Vec4i> hierarchy;  // 定义一个向量，用于存储图像中检测到的所有轮廓的层级信息
  cv::findContours(
      binary_img, contours, hierarchy, cv::RETR_EXTERNAL,
      cv::CHAIN_APPROX_SIMPLE);  // 仅检索最外层轮廓（忽略嵌套轮廓），仅保留水平、垂直和对角方向的端点（矩形仅需4个点）

  vector<Light> lights;
  debug_data_.lights.data.clear();

  for (const auto& contour : contours)
  {
    if (contour.size() < 5)
    {
      continue;  // 跳过轮廓点数太少的，避免误判，减小干扰
    }

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (IsLight(light))
    {
      auto rect = light.boundingRect();
      if (  // Avoid assertion failed 确保矩形区域完全在图像范围内，避免越界访问
          0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols &&
          0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= rbg_img.rows)
      {
        int sum_r = 0;
        int sum_b = 0;
        auto roi = rbg_img(rect);  // 创建一个指向rbg_img图像rect区域的一张新图ROI
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++)
        {
          for (int j = 0; j < roi.cols; j++)
          {
            if (cv::pointPolygonTest(contour,
                                     cv::Point2f(static_cast<float>(j + rect.x),
                                                 static_cast<float>(i + rect.y)),
                                     false) >= 0)
            {  // 函数用于判断一个点是否在给定的轮廓（多边形）内(>0)，或者在轮廓上(=0)，亦或是在轮廓外(<0)。
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ? 判断红蓝灯条
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool Detector::IsLight(const Light& light)
{
  // The ratio of light (short side / long side) 通关宽高比判断是否是灯条
  double ratio = light.width / light.length;
  bool ratio_ok = params_.l.min_ratio < ratio && ratio < params_.l.max_ratio;

  bool angle_ok = light.tilt_angle < params_.l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.center_x = static_cast<int>(light.center.x);
  light_data.ratio = static_cast<float>(ratio);
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  debug_data_.lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> Detector::MatchLights(const std::vector<Light>& lights)
{
  std::vector<Armor> armors;
  debug_data_.armors.data.clear();

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++)
  {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++)
    {
      if (light_1->color != params_.detect_color ||
          light_2->color != params_.detect_color)
      {
        continue;
      }

      if (ContainLight(*light_1, *light_2, lights))
      {
        continue;
      }

      auto type = IsArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID)
      {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
// 判断是否存在干扰灯条
bool Detector::ContainLight(const Light& light_1, const Light& light_2,
                            const std::vector<Light>& lights)
{
  // 1. 创建装甲板：用两个灯条的顶端和底端点构建最小外接矩形
  auto points =
      std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);  // 生成整数坐标的矩形

  // 2. 遍历所有灯条进行检查
  for (const auto& test_light : lights)
  {
    // 跳过当前正在配对的两个灯条（通过中心点坐标比较）
    if (test_light.center == light_1.center || test_light.center == light_2.center)
    {
      continue;
    }

    // 3. 检查其他灯条的关键点是否在装甲板内
    if (bounding_rect.contains(test_light.top) ||     // 顶点在区域内
        bounding_rect.contains(test_light.bottom) ||  // 底点在区域内
        bounding_rect.contains(test_light.center))
    {               // 中心点在区域内
      return true;  // 发现干扰灯条立即返回
    }
  }

  return false;  // 遍历完成未发现干扰灯条
}

ArmorType Detector::IsArmor(const Light& light_1, const Light& light_2)
{
  // Ratio of the length of 2 lights (short side / long side) 灯条长度比例检查
  double light_length_ratio = light_1.length < light_2.length
                                  ? light_1.length / light_2.length
                                  : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > params_.a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length) 灯条中心距离检查
  double avg_light_length = (light_1.length + light_2.length) / 2;
  double center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (params_.a.min_small_center_distance <= center_distance &&
                             center_distance < params_.a.max_small_center_distance) ||
                            (params_.a.min_large_center_distance <= center_distance &&
                             center_distance < params_.a.max_large_center_distance);

  // Angle of light center connection  灯条中心连线角度检查
  cv::Point2f diff = light_1.center - light_2.center;
  double angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < params_.a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type{};
  if (is_armor)
  {
    type = center_distance > params_.a.min_large_center_distance ? ArmorType::LARGE
                                                                 : ArmorType::SMALL;
  }
  else
  {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = static_cast<int>((light_1.center.x + light_2.center.x) / 2);
  armor_data.light_ratio = static_cast<float>(light_length_ratio);
  armor_data.center_distance = static_cast<float>(center_distance);
  armor_data.angle = static_cast<float>(angle);
  debug_data_.armors.data.emplace_back(armor_data);

  return type;
}

// 将检测到的所有装甲板上的数字图像垂直拼接成一个单独的图像并返回
const cv::Mat& Detector::GetNumbersImage()
{
  std::vector<cv::Mat> number_imgs;
  number_imgs.reserve(armors_.size());
  for (const auto& armor : armors_)
  {
    if (!armor.number_img.empty())
    {
      number_imgs.emplace_back(armor.number_img);
    }
  }
  if (number_imgs.empty())
  {
    all_num_img_.release();
  }
  else
  {
    cv::vconcat(number_imgs, all_num_img_);
  }
  return all_num_img_;
}

void Detector::DrawResults(cv::Mat& img)
{
  // Draw Lights
  for (const auto& light : lights_)
  {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color =
        (light.color == RED) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto& armor : armors_)
  {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0),
             2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0),
             2);
  }

  // Show numbers and confidence
  for (const auto& armor : armors_)
  {
    cv::putText(img, armor.classfication_result, armor.left_light.top,
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  }
}

const DebugData& Detector::GetDebugData()
{
  // Sort lights and armors data by x coordinate
  std::sort(debug_data_.lights.data.begin(), debug_data_.lights.data.end(),
            [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
  std::sort(debug_data_.armors.data.begin(), debug_data_.armors.data.end(),
            [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });
  return debug_data_;
}

}  // namespace rm_auto_aim
