#include "include/HikCamera.hpp"
#include "include/Detector.hpp"
#include "include/Armor.hpp"
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
    // io::HikCamera Hik(5,0.5,16);
    cv::namedWindow("hh");
    cv::namedWindow("result");
    cv::namedWindow("debug");

    Detector indentifyAomor(Light::Color::Red,0.5,"/home/king/Desktop/SinAim_rm/10.15.13/workindentify/rm-main/model/mlp.onnx");
    
    while(true)
    {
        io::HikCamera::ImageData frame; 
        // Hik.read(frame);
        frame.image = cv::imread("/home/king/Desktop/SinAim_rm/10.15.13/workindentify/images/frame2.png");
        cv::Mat debug = indentifyAomor.preprocessImage(frame.image);
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        std::vector<Armor> armors = indentifyAomor.DectectedArmor(frame.image);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        total_elapsed_seconds += end - start;
        num++;

        if(num%100==0&&num!=0)
        {
            std::cout<<total_elapsed_seconds.count()/num<<std::endl;
        }
        
        indentifyAomor.ArmorShow(frame.image, armors);
        cv::imshow("hh",frame.image);
        cv::imshow("result",frame.image);
        cv::imshow("debug",debug);
        cv::waitKey(1);
    }
}