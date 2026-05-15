#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_base.hpp"
#include "armor_detector/light_corner_corrector.hpp"
#include "armor_detector/number_classifier.hpp"

namespace rclcpp
{
class Node;
}

namespace rm_auto_aim
{

class Detector : public DetectorBase
{
 public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle与垂直方向的最大差角
    double max_angle;
    // area condition
    double min_fill_ratio;
  };

  struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  struct ClassifierParams
  {
    std::string model_path;
    std::string label_path;
    double threshold;
    std::vector<std::string> ignore_classes;
  };

  struct CornerCorrectorParams
  {
    bool use_corner_corrector;
    double max_brightness;
    double scale;
    double start;
    double end;
  };

  struct DetectorParams
  {
    int binary_lower_thres;
    int binary_upper_thres;
    int detect_color;
    LightParams l;
    ArmorParams a;
    ClassifierParams c;
    CornerCorrectorParams cc;
  };

  static std::unique_ptr<Detector> Create(rclcpp::Node& node);

  explicit Detector(DetectorParams& params);

  DetectionResult Detect(const cv::Mat& input) override;

  cv::Mat PreprocessImage(const cv::Mat& input);
  std::vector<Light> FindLights(const cv::Mat& rbg_img, const cv::Mat& binary_img) noexcept;
  std::vector<Armor> MatchLights(const std::vector<Light>& lights);

  void DrawResults(cv::Mat& img) override;

 private:
  bool IsLight(const Light& possible_light);
  bool ContainLight(const Light& light_1, const Light& light_2,
                    const std::vector<Light>& lights);
  ArmorType IsArmor(const Light& light_1, const Light& light_2);
  const DebugData& GetDebugData();
  const cv::Mat& GetNumbersImage();

  DetectorParams params_;
  std::unique_ptr<NumberClassifier> classifier_;
  std::unique_ptr<LightCornerCorrector> light_corner_corrector_ = nullptr;
  std::vector<Light> lights_;
  std::vector<Armor> armors_;


  // Debug msgs
  cv::Mat binary_img_;
  cv::Mat gray_img_;
  cv::Mat all_num_img_;
  DebugData debug_data_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
