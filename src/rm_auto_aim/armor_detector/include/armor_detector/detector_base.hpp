#ifndef ARMOR_DETECTOR__DETECTOR_BASE_HPP_
#define ARMOR_DETECTOR__DETECTOR_BASE_HPP_

#include <opencv2/core/mat.hpp>
#include <optional>
#include <vector>

#include "armor_detector/armor.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"
namespace rm_auto_aim
{
struct DebugData
{
  auto_aim_interfaces::msg::DebugLights lights;
  auto_aim_interfaces::msg::DebugArmors armors;
};

struct DetectionResult
{
  std::vector<Armor> armors;
  std::optional<cv::Mat> binary_image;
  std::optional<DebugData> debug_data;
  std::optional<cv::Mat> numbers_image;
};

class DetectorBase
{
 public:
  virtual ~DetectorBase() = default;
  virtual DetectionResult Detect(const cv::Mat& input) = 0;
  virtual void DrawResults(cv::Mat& img) = 0;
  std::vector<std::pair<std::string, uint64_t>> debug_latencies_;  // 各环节延迟
  const std::vector<std::pair<std::string, uint64_t>>& GetDebugLatencies() const
  {
    return debug_latencies_;
  }
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_BASE_HPP_
