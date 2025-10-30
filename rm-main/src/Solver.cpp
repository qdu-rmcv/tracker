#include "../include/Solver.hpp"
#include <array>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
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

    // 相机到云台的旋转矩阵 (Rotation Matrix from Camera to Gripper)
    fs["R_Cam_to_gripper"] >> this->R_Cam_to_gripper;

    // 相机到云台的平移矩阵 (Translation Matrix from Camera to Gripper)
    fs["T_Cam_to_gripper"] >> this->T_Cam_to_gripper;

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
        // std::cerr<<reprojectionError.front()<<" "<<reprojectionError.back()<<std::endl;
        //筛选歧义解
        double Z_data[3]{0,0,10};
        cv::Mat Z_vector(cv::Size(1,3),CV_64FC1,Z_data);

        cv::Mat r_0,r_1;
        cv::Rodrigues(rvecs.front(), r_0);
        cv::Rodrigues(rvecs.back(), r_1);

        cv::Mat Z_camera_0 = r_0 * Z_vector + tvecs.front();
        cv::Mat Z_camera_1 = r_1 * Z_vector + tvecs.back();

        cv::Mat R,T;
        // std::cerr<<Z_camera_0.at<double>(2,0)<<" "<<Z_camera_1.at<double>(2,0)<<std::endl;
        if(Z_camera_0.at<double>(2,0) > 0) {R = r_0; T = tvecs.front();}
        else {R = r_1; T = tvecs.back();}

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

void Solver::ConverToWorld(ArmorPosi& armor_posi, const cv::Quatd& q_gripper_to_world)
{
    cv::Mat R(q_gripper_to_world.toRotMat3x3());// 手坐标系到世界坐标系的旋转矩阵
    
    for (auto& pos : armor_posi.posi) 
    {
        // 将装甲板位置从相机坐标系转换到手坐标系
        cv::Mat P = this->R_Cam_to_gripper * cv::Mat(cv::Point3d(pos.x, pos.y, pos.z)) + this->T_Cam_to_gripper;
        
        // 将装甲板位置从手坐标系转换到世界坐标系
        P = R * P;

        // 更新装甲板位置
        pos = cv::Point3d(P.at<double>(0, 0), P.at<double>(1, 0), P.at<double>(2, 0));
    }
}


void Solver::ansShow(const cv::Point3d& posi,cv::Mat& image)
{
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F); // 单位旋转向量
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F); // 单位平移向量

    // 3. 执行投影
    // cv::projectPoints 需要一个点的向量作为输入
    std::vector<cv::Point3d> objectPoints;
    objectPoints.push_back(posi);

    // 用于存储投影结果的2D点向量
    std::vector<cv::Point2d> imagePoints;

    //重投影
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    //在图像上绘制结果
    // 输出和可视化结果
    // 投影后的2D点坐标
    cv::Point2d projectedPoint = imagePoints[0];
    int imageWidth = image.rows;
    int imageHeight = image.cols;

    // 在图像上绘制投影点 (画一个红色的圆圈)
    // 检查点是否在图像范围内
    if (projectedPoint.x >= 0 && projectedPoint.x < imageWidth &&
        projectedPoint.y >= 0 && projectedPoint.y < imageHeight)
    {
        cv::circle(image, projectedPoint, 5, cv::Scalar(0, 0, 255), -1); // 红色实心圆
        cv::putText(image, "Projected Point", cv::Point(projectedPoint.x + 10, projectedPoint.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    } else {
        std::cout << "Projected point is outside the image frame." << std::endl;
    }
    // 显示图像
    cv::imshow("Projected Point Visualization", image);
    cv::waitKey(1); // 等待按键后退出
}
