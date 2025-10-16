#include "../include/Solver.hpp"
#include <array>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <vector>

Solver::Solver(std::string config_path)
{
    // this->cameraMatrix(3,3);
    // this->distCoeffs(5,1);
    cv::FileStorage fs;
    if (!fs.open(config_path, cv::FileStorage::READ)) 
        std::cerr << "Error: Failed to open YAML file: " << config_path << std::endl;
    
    std::cout << "Successfully opened " << config_path << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    cameraMatrix = cv::Mat_<double>(3,3);
    distCoeffs = cv::Mat_<double>(5,1);

    fs["camera_matrix"] >> this->cameraMatrix;
    fs["distortion_coeffs"] >> this->distCoeffs;
    // std::cout<<cameraMatrix.size<<std::endl;

    
    for(int i=0;i<4;i++)
    {
        this->objectBigArmor[i] = cv::Mat_<double>(3, 1);
        this->objectSmallArmor[i] = cv::Mat_<double>(3, 1);
    }

    this->objectBigArmor[0]<< 0, 0, 0;
    this->objectBigArmor[1]<< 230, 0, 0;
    this->objectBigArmor[2]<< 230, 55, 0;
    this->objectBigArmor[3]<< 0, 55, 0;

    this->objectSmallArmor[0]<< 0, 0, 0;
    this->objectSmallArmor[1]<< 135, 0, 0;
    this->objectSmallArmor[2]<< 135, 55, 0;
    this->objectSmallArmor[3]<< 0, 55, 0;


}

std::vector<ArmorPosi> Solver::SolvePnP(const std::vector<Armor>& armors)
{
    std::vector<ArmorPosi> armors_posi;
    if(armors.empty()) return armors_posi;
    armors_posi.reserve(armors.size());

    for(const auto& armor:armors)
    {
        std::vector<cv::Mat> rvecs,tvecs;
        std::vector<double> reprojectionError;

        if(armor.type == Armor::Type::one)//区分大小装甲板
        int solutions = cv::solvePnPGeneric(
            this->objectBigArmorP,
            armor.Lightcorners,
            cameraMatrix,
            distCoeffs,
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE, // 使用 IPPE 算法获取多个解
            cv::noArray(),
            cv::noArray(),
            reprojectionError
        );
        else{
        int solutions = cv::solvePnPGeneric(
            this->objectSmallArmorP,
            armor.Lightcorners,
            cameraMatrix,
            distCoeffs,
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE, // 使用 IPPE 算法获取多个解
            cv::noArray(),
            cv::noArray(),
            reprojectionError
        );}
        // if(reprojectionError[0]>10||reprojectionError[1]) continue;

        //筛选歧义解
        double Z_data[3]{0,0,10};
        cv::Mat Z_vector(cv::Size(1,3),CV_64FC1,Z_data);

        cv::Mat r_0,r_1;
        cv::Rodrigues(rvecs[0], r_0);
        cv::Rodrigues(rvecs[1], r_1);

        cv::Mat Z_camera_0 = r_0 * Z_vector + tvecs[0];
        cv::Mat Z_camera_1 = r_1 * Z_vector + tvecs[1];

        cv::Mat R,T;
        std::cerr<<Z_camera_0.at<double>(2,0)<<" "<<Z_camera_1.at<double>(2,0)<<std::endl;
        if(Z_camera_0.at<double>(2,0) > 0) {R = r_0; T = tvecs[0];}
        else {R = r_1; T = tvecs[1];}

        std::array<cv::Point3d,4> posi;
        for(int i=0;i<4;i++)
        {
            if(armor.type==Armor::Type::one){
                cv::Mat P = R * objectBigArmor[i] + T;
                posi[i] = cv::Point3d(P.at<double>(0,0),P.at<double>(1,0),P.at<double>(2,0));
            }else {
                cv::Mat P = R * objectSmallArmor[i] + T;
                posi[i] = cv::Point3d(P.at<double>(0,0),P.at<double>(1,0),P.at<double>(2,0));
            } 
        }
        armors_posi.emplace_back(posi,armor.type);//记录
    }
    return armors_posi;
}
