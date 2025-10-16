#ifndef SOLVER_CLASS_INCLUDE
#define SOLVER_CLASS_INCLUDE
#include "Armor.hpp"
#include "string"
#include "opencv2/opencv.hpp"
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

private:
    cv::Mat_<double> cameraMatrix;
    cv::Mat_<double> distCoeffs;
    std::vector<cv::Point3d> objectPoints;
};

#endif