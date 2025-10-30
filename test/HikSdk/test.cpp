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
#include <thread>
#include <vector>

static int num=0;
static std::chrono::duration<double> total_elapsed_seconds(0.0);
using namespace std::chrono_literals;
int main()
{
    io::HikCamera Hik(3,10);
    // Hik.continueCap(3);
    cv::namedWindow("hh");
    cv::namedWindow("result");
    cv::namedWindow("debug");

    Detector indentifyAomor(Light::Color::Red,0.5,"/home/king/desktop/SinAim_rm/10.16/rm-main/model/mlp.onnx");
    Solver Sov("/home/king/desktop/SinAim_rm/10.16/config/Solver_config.yaml");


    io::HikCamera::ImageData frame;
while (true) {

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    Hik.read(frame);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"capture time: "<<(end - start).count()*1e-6<<" ms"<<std::endl;

    if(!frame.image.empty())
    {
    // frame.image = cv::imread("/home/king/Desktop/SinAim_rm/10.15.13/workindentify/images/frame2.png");
    // cv::Mat debug = indentifyAomor.preprocessImage(frame.image);

        
    //     std::vector<Armor> armors = indentifyAomor.DectectedArmor(frame.image);
    //     std::vector<ArmorPosi> armors_posi = Sov.SolvePnP(armors);

        

        // indentifyAomor.ArmorShow(frame.image, armors);
        cv::imshow("hh",frame.image);
        // cv::imshow("result",frame.image);
        // cv::imshow("debug",debug);
    //     cv::waitKey(10);
    // std::this_thread::sleep_for(ms);    
}else {
  std::cerr<<"here"<<std::endl;
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