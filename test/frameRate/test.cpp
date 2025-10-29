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

using namespace std::chrono_literals;
static int num=0;
static int Armor_num=0;
static std::chrono::duration<double> total_elapsed_seconds(0.0);
static std::chrono::duration<double> delay_seconds(0.0);
static std::chrono::duration<double> total_delay_seconds(0.0);
int main()
{
    Detector indentifyAomor(Light::Color::Red,0.5,"../../../rm-main/model/mlp.onnx");
    Solver Sov("../../../config/Solver_config.yaml");
    io::HikCamera Hik(3,10);
    Hik.continueCap(5);
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    while(true)
    {
        io::HikCamera::ImageData frame; 
        Hik.read(frame);
        num++;

        std::chrono::steady_clock::time_point read_time = std::chrono::steady_clock::now();
        if(delay_seconds<(read_time-frame.time)) delay_seconds = read_time-frame.time;
        total_delay_seconds += read_time-frame.time;

        // std::vector<Armor> armors = indentifyAomor.DectectedArmor(frame.image);
        // std::vector<ArmorPosi> armors_posi = Sov.SolvePnP(armors);
        // Armor_num += armors_posi.size();

        if(num%1000==0&&num!=0)
        {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            total_elapsed_seconds = end - start;
            
            std::cout<<"最高读取延迟："<< delay_seconds.count()<<std::endl;
            std::cout<<"平均读取延迟："<< total_delay_seconds.count()/1000<<std::endl;
            std::cout<<"帧率："<<num/total_elapsed_seconds.count()<<std::endl;
            std::cout<<"检测到装甲板数量："<<Armor_num<<std::endl;

            num = 0;
            Armor_num = 0;
            start = std::chrono::steady_clock::now();
            delay_seconds = 0ms;
            total_delay_seconds = 0ms;
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