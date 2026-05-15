#ifndef ARMOR_DETECTOR__DETECTOR_FACTORY_HPP_
#define ARMOR_DETECTOR__DETECTOR_FACTORY_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "armor_detector/detector.hpp"
#include "armor_detector/detector_base.hpp"
#include "armor_detector/yolo_detector.hpp"

namespace rclcpp
{
class Node;
}

namespace rm_auto_aim
{

inline std::unique_ptr<DetectorBase> create_detector(const std::string& type,
                                                     rclcpp::Node& node)
{
  if (type == "traditional")
  {
    return Detector::Create(node);
  }

#if ARMOR_DETECTOR_HAS_OPENVINO || ARMOR_DETECTOR_HAS_TENSORRT
  if (type == "yolo")
  {
    return YoloDetector::Create(node);
  }
#else
  if (type == "yolo")
  {
    throw std::runtime_error(
        "yolo detector is requested, but this package was built without OpenVINO "
        "support");
  }
#endif

  throw std::runtime_error("unknown detector type: " + type);
}

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_FACTORY_HPP_
