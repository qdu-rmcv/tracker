#ifndef SOLVER_CLASS_INCLUDE
#define SOLVER_CLASS_INCLUDE
#include "Armor.hpp"
#include "string"
#include "opencv2/opencv.hpp"
#include <opencv2/core/base.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
class Solver
{
public:

    Solver(std::string config_path);
    std::vector<ArmorPosi> SolvePnP(const std::vector<Armor>& armors);
    void ConverToWorld(ArmorPosi armor_posi);
    void ansShow(const cv::Point3d& posi,cv::Mat& image);

private:
    cv::Mat_<double> cameraMatrix;
    cv::Mat_<double> distCoeffs;
    std::array<cv::Mat_<double>,4> objectBigArmor;
    std::array<cv::Mat_<double>,4> objectSmallArmor;
    std::vector<cv::Point3f> objectBigArmorP{{0,0,0},{230,0,0},{230,55,0},{0,55,0}};
    std::vector<cv::Point3f> objectSmallArmorP{{0,0,0},{135,0,0},{135,55,0},{0,55,0}};
};

#endif