#include "../../rm-main/include/HikCamera.hpp"
#include "../../rm-main/include/Detector.hpp"
#include "../../rm-main/include/Armor.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <string>
#define Debug
using namespace std::chrono_literals;
static int num=1722;
static int framenum=0;
static int Armor_num=0;
static std::chrono::duration<double> total_elapsed_seconds(0.0);
static std::chrono::duration<double> delay_seconds(0.0);
static std::chrono::duration<double> total_delay_seconds(0.0);

// 2. 定义裁剪参数
int cropWidth = 112;
int cropHeight = 112;

// 3. 定义步长 (Stride)
// 如果 step = 112，则是“无重叠”裁剪（平铺）
// 如果 step < 112，则是“重叠”裁剪（Sliding Window）
int stepX = 112; 
int stepY = 112;

    std::string saveDir = "/home/king/Pytorch/train/data/train/7_negetive/";
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

            imshow("Current Crop", frame.image);
            cv::waitKey(1); 

            
            // 定义感兴趣区域 (Region of Interest)
 
            
            // 获取裁剪图像
            cv::Mat crop ;
            cv::resize(frame.image,crop,cv::Size(112,112));

            // 5. 保存图像
            // 生成文件名: crops/crop_0.jpg, crops/crop_1.jpg ...
            // 也可以包含坐标信息: crop_x_y.jpg
            std::string filename = saveDir + "image_" + std::to_string(num++) + ".png";
            
            // 这里的 clone() 是为了确保数据独立，如果只是显示可以不用
            // imwrite(filename, crop); 
            
            // 可选：实时显示裁剪过程

    


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