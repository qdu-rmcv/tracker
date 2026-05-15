#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class NumberClassifier
{
 public:
  NumberClassifier(const std::string& model_path, const std::string& label_path,
                   const double& thre,
                   const std::vector<std::string>& ignore_classes = {});

  void ExtractNumbers(const cv::Mat& src, std::vector<Armor>& armors);

  void Classify(std::vector<Armor>& armors);

  void SetThreshold(double threshold);

 private:
  double threshold_;
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
