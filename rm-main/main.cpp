#include "include/HikCamera.hpp"
#include "include/Detector.hpp"
#include "include/Armor.hpp"
#include "include/Solver.hpp"
#include <opencv2/core/mat.hpp>
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
    cv::namedWindow("hh");
    cv::namedWindow("result");
    cv::namedWindow("debug");

    Detector indentifyAomor(Light::Color::Red,0.5,"/home/king/desktop/SinAim_rm/10.16/rm-main/model/mobilenet_v3_112_rgb.onnx");
    Solver Sov("/home/king/desktop/SinAim_rm/10.16/config/Solver_config.yaml");
    
    while(true)
    {
        io::HikCamera::ImageData frame; 
        Hik.read(frame);
        // frame.image = cv::imread("/home/king/Desktop/SinAim_rm/10.15.13/workindentify/images/frame2.png");
        cv::Mat debug = indentifyAomor.preprocessImage(frame.image);

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        std::vector<Armor> armors = indentifyAomor(frame.image);
        std::vector<ArmorPosi> armors_posi = Sov.SolvePnP(armors);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        
        total_elapsed_seconds += end - start;
        num++;

        if(num%100==0&&num!=0)
        {
            std::cout<<total_elapsed_seconds.count()/num<<std::endl;
        }
        if(!armors_posi.empty())
            std::cout<<armors_posi[0].posi[0]<<std::endl;
        indentifyAomor.ArmorShow(frame.image, armors);
        cv::imshow("hh",frame.image);
        cv::imshow("result",frame.image);
        cv::imshow("debug",debug);
        cv::waitKey(1);
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