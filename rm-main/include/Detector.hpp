#ifndef DETECTOR_CLASS
#define DETECTOR_CLASS
#include "Armor.hpp"
#include "NumClassifier.hpp"
#include <deque>
#include <string>
#include <vector>
class Detector {
public:
    Detector(Light::Color color,double confidence,std::string model_path);

    std::vector<Armor> DectectedArmor(cv::Mat& frame);
    void ArmorShow(cv::Mat & rgb_img, const std::deque<Armor> & armors);
    void ArmorShow(cv::Mat & rgb_img, const std::vector<Armor> & armors);

public:
    double confidence;
    Light::Color color;
    cv::Mat gray_img;

private:
    NumClassifier classifier;
public:
    cv::Mat preprocessImage(cv::Mat& rgb_img); //图像预处理
    std::deque<Light> FindLight(const cv::Mat & rgb_img, const cv::Mat & binary_img); //寻找灯条
    std::deque<Armor> FindArmor(const std::deque<Light> & lights); //寻找装甲板
    std::vector<Armor> ClassifyArmor(const std::deque<Armor>& armors);
    std::vector<cv::Mat> ROIArmor(const std::deque<Armor>& armors);
};
#endif