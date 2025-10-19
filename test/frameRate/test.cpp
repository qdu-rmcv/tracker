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
#include <vector>

static int num=0;
static std::chrono::duration<double> total_elapsed_seconds(0.0);
int main()
{
    io::HikCamera Hik(5,3,10);
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    while(true)
    {
        io::HikCamera::ImageData frame; 
        Hik.read(frame);
        num++;

        if(num%1000==0&&num!=0)
        {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            total_elapsed_seconds = end - start;
            std::cout<<num/total_elapsed_seconds.count()<<std::endl;
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