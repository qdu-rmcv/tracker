#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <string>
#include <vector>

class NumClassifier
{
public:
    struct Ans{
        int id;
        float confidence;
        Ans(int id,double con):id(id),confidence(con){}
    };
    NumClassifier(std::string model_path);
    std::vector<Ans> Classify(const std::vector<cv::Mat>& armors_pattern);

private:
    cv::dnn::Net Net;
};




#endif