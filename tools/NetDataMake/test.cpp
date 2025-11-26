#include "../../rm-main/include/HikCamera.hpp"
#include "../../rm-main/include/Detector.hpp"
#include "../../rm-main/include/Armor.hpp"
#include "../../rm-main/include/Solver.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <deque>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <string>
#include <utility>
#include <vector>
#define Debug
using namespace std::chrono_literals;
static int num=623;
static int Armor_num=0;
static std::chrono::duration<double> total_elapsed_seconds(0.0);
static std::chrono::duration<double> delay_seconds(0.0);
static std::chrono::duration<double> total_delay_seconds(0.0);
int main()
{
    Detector detect(Light::Color::Blue,0.3,"../../../rm-main/model/mobilenet_v3_112_rgb.onnx");
    io::HikCamera Hik(3,17);
    Hik.continueCap(5);
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    // cv::namedWindow("gray_img",cv::WINDOW_NORMAL);
    while(true)
    {
        io::HikCamera::ImageData frame; 

        Hik.read(frame);
        
        if(frame.image.empty()) continue;

        cv::Mat gray_img = detect.preprocessImage(frame.image);

        
        auto lights  = detect.FindLight(gray_img);
        #ifdef Debug
        std::cout <<"lights num:" << lights.size() << "\n";
        #endif
        auto armors = detect.FindArmor(lights);
        #ifdef Debug
        std::cout <<"armors num:" << armors.size() << "\n";
        #endif
        auto roi = detect.ROIArmor( armors );


        detect.ArmorShow(frame.image, armors);
        cv::imshow("gray_img",frame.image);
        cv::waitKey(1);

        for(const auto& img : roi )
        {
            cv::imshow("roi",img);
            // cv::imwrite("/home/king/Pytorch/train/data/train/5_guard/image_"+std::to_string(num++)+".png", img);
            cv::waitKey(1);
        }
    }
}
    // std::string config_path = "/home/king/desktop/SinAim_rm/10.16/config/Solver_config.yaml";

    // cv::FileStorage fs;
    // if (!fs.open(config_path, cv::FileStorage::READ)) 
    //     std::cerr << "Error: Failed to open YAML file: " << config_path << std::endl;
    
    // std::cout << "Successfully opened " << config_path << std::endl;
    // std::cout << "------------------------------------------" << std::endl;

    // cv::Mat_<double> cameraMatrix;
    // cv::Mat_<double> distCoeffs;
    // fs["camera_matrix"] >> cameraMatrix;
    // fs["distortion_coeffs"] >> distCoeffs;

    // for(int i=0;i<1;i++)
    // {
    //     for(int j=0;j<5;j++)
    //     {
    //             std::cout<<distCoeffs(i,j)<<" ";
    //     }
    //     std::cout<<std::endl;
    // }