#if ARMOR_DETECTOR_ENABLE_YOLO

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_detector/yolo_detector.hpp"

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " <image_path> <model_path> [detect_color=0]"
              << '\n';
    std::cerr << "  detect_color: 0=RED, 1=BLUE" << '\n';
    return 1;
  }

  std::string image_path = argv[1];
  std::string model_path = argv[2];
  int detect_color = (argc > 3) ? std::atoi(argv[3]) : 0;

  cv::Mat bgr_img = cv::imread(image_path);
  if (bgr_img.empty())
  {
    std::cerr << "Failed to load image: " << image_path << '\n';
    return 1;
  }
  cv::imshow("Input Image", bgr_img);

  // 初始化检测器
  rm_auto_aim::YoloDetector::YoloParams config = {.model_path = model_path,
                                                  .device = "CPU",
                                                  .score_threshold = 0.7f,
                                                  .min_confidence = 0.8f,
                                                  .nms_threshold = 0.3f,
                                                  .ignore_classes = {"negative"},
                                                  .detect_color = detect_color};

  rm_auto_aim::YoloDetector detector(config);

  auto detect_result = detector.Detect(bgr_img);

  std::cout << "Detected " << detect_result.armors.size() << " armor(s):" << '\n';
  for (const auto& armor : detect_result.armors)
  {
    std::cout << "  " << armor.classfication_result
              << "  type=" << rm_auto_aim::ARMOR_TYPE_STR[static_cast<int>(armor.type)]
              << "  center=(" << armor.center.x << ", " << armor.center.y << ")" << '\n';
  }

  detector.DrawResults(bgr_img);

  cv::imshow("YoloDetector Result", bgr_img);
  cv::waitKey(0);

  return 0;
}

#endif