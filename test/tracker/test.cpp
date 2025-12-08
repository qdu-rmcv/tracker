#include "../../rm-main/include/HikCamera.hpp"
#include "../../rm-main/include/Detector.hpp"
#include "../../rm-main/include/Armor.hpp"
#include "../../rm-main/include/Solver.hpp"
#include "../../rm-main/include/Tracker.hpp"
#include <chrono>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

int main()
{
    Detector detect(Light::Color::Red,0.5,"/home/king/AUTO-Aming-system/rm-main/model/mobilenet_v3_112_rgb.onnx");
    Solver Sov("../../../config/Solver_config.yaml");
    Tracker track;

    io::HikCamera Hik(1,17);
    Hik.continueCap(3);
    int frame_count = 0;
    // cv::namedWindow("gray_img",cv::WINDOW_NORMAL);
    auto start = std::chrono::steady_clock::now();
    while(true)
    {
        io::HikCamera::ImageData frame; 
            
        Hik.read(frame);

        if(frame.image.empty()) continue;
        frame_count++;
        if(frame_count % 200 == 0 && frame_count != 0)
        {
            auto end = std::chrono::steady_clock::now();
            auto elapsed_seconds = end - start;
            std::cout << "Average FPS over last 200 frames: " << 200.0 / (elapsed_seconds.count()*1e-9) << std::endl;
            start = std::chrono::steady_clock::now();
            
        }
// continue;

        auto binary_img = detect.preprocessImage(frame.image); //预处理图像

        auto lights = detect.FindLight(binary_img); //寻找灯条

        auto possible_armors = detect.FindArmor(lights); //寻找装甲板


        //将装甲板转换为vector
        std::vector<Armor> armors;
        armors.reserve(possible_armors.size());
        for(const auto& armor : possible_armors)
        {
            armors.push_back(armor);
        }

        detect.ArmorShow(frame.image, armors);

        if(armors.empty()) {continue;}
        armors[0].confidence = 1.0;
        armors[0].type = Armor::Type::guard;
        //解算装甲板位置
        auto armors_posi = Sov(armors);

        //追踪装甲板
        if(armors_posi.empty()) continue;

        std::cout << "Predicted Position: " << armors_posi[0].posi << std::endl;
        Eigen::Matrix<double, 3, 1> posi;
        posi << armors_posi[0].posi.x/10, armors_posi[0].posi.y/10, armors_posi[0].posi.z/10;

        auto ans = track(posi,0.02);
        std::cout<< "Filtered Position: " << ans.transpose() << std::endl;

        double dt = 0.3;

        cv::Point3d predict_posi;
        predict_posi.x = (ans(0,0) + dt * ans(3,0)) * 10;
        predict_posi.y = (ans(1,0) + dt * ans(4,0)) * 10;
        predict_posi.z = (ans(2,0) + dt * ans(5,0)) * 10;

        std::cout << "Predicted Position: " << predict_posi << std::endl;
        Sov.ansShow(predict_posi, frame.image);

    }
}
