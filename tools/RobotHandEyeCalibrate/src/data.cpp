#include "HikCamera.hpp"
#include "fast_queue.hpp"
#include "TimeMatcher.hpp"

#include <opencv2/core/quaternion.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/matx.hpp>
#include <string>
#include <iomanip>    // 用于格式化时间字符串
#include <sstream>    // 用于构建字符串
#include <thread>

// 函数：生成一个基于当前时间的唯一文件名
static int Num=0;

std::string generate_filename(int &Num) {
    
    std::string ss;
    ss ="image_"+std::to_string(Num++)+"_";
    return ss;
}

int main() {
    // 1. 定义数据保存目录
    std::string output_dir = "../Data/images";
    std::string config_path = "../Data/Calibration_R_T.yaml";

    // 3. 打开默认摄像头 
    io::HikCamera Hik(2,10,false);
    io::HikCamera::ImageData frame; 


    std::cout << "摄像头已成功打开。按 '空格键' 截图，按 'ESC' 退出。" << std::endl;

    //加载存储数据的YAML文件
    cv::FileStorage fs;
    if (!fs.open(config_path, cv::FileStorage::WRITE)) {
        std::cerr << "Error: Failed to open YAML file: " << config_path << std::endl;
        return -1;  // 失败时返回
    }


    std::cout << "Successfully opened " << config_path << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    // 5. 创建一个窗口用于显示
    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);


    //采集imu数据线程
    // IMU数据队列

    struct IMUData {
        cv::Quatd orientation;
        std::chrono::steady_clock::time_point time;
    };
    FastQueue<IMUData> imu_queue{3};
    std::thread imu_thread([&imu_queue]() mutable {
        while (true) {
            // 采集IMU数据
            // IMUData imu_data = ...;  // 获取IMU数据
            // bool full = imu_queue.push(imu_data);
            //if(full) {
            //    std::cerr << "IMU queue is full, Hik don't work!" << std::endl;
            //}
        }
    });


    //初始化匹配器
    TimeMatcher<io::HikCamera::ImageData, IMUData> matcher(2+0.3+4,1);
    matcher.setFrontwave(0.5,1);
    matcher.setBackwave(0.5,1);


    struct MatchedData {
        io::HikCamera::ImageData image;
        IMUData imu;
    };
    FastQueue<MatchedData> matched_queue{10};

    // 相机采集和时间匹配线程
    std::thread camera_thread
    (
        [&]() mutable
        {
            while (true) 
            {
                // 6. 从摄像头捕获一帧
                std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                Hik.read(frame);

                if (frame.image.empty()) continue;
                cv::imshow("Camera Feed", frame.image);
                cv::waitKey(1);
                auto read_time = std::chrono::steady_clock::now()- start;

                if(read_time < std::chrono::milliseconds(1)) continue;//不是最新帧，跳过

                // 进行时间匹配
                while (true) 
                {
                    if(imu_queue.empty()) 
                    {
                        std::cerr<<"imu queue empty"<<std::endl;
                        break;
                    }

                    if(std::chrono::steady_clock::now() - imu_queue.front().time > std::chrono::milliseconds(15))
                    {
                        // imu数据过旧，丢弃
                        imu_queue.pop();
                        continue;
                    }

                    auto result = matcher.Match(frame,imu_queue.front());
                    if(result == TimeMatcher<io::HikCamera::ImageData, IMUData>::Result::FrontTout)
                    {
                        // image超时，等待下一帧
                        std::cerr<<"image time out"<<std::endl;
                        break;
                    }
                    else if(result == TimeMatcher<io::HikCamera::ImageData, IMUData>::Result::BackTout)
                    {
                        // 后端超时，尝试获取更多IMU数据
                        imu_queue.pop();
                        continue;
                    }

                    // 成功匹配
                    matched_queue.push(MatchedData{frame, imu_queue.front()});
                    imu_queue.pop();
                    break;
                }
            }
        }
    );


        // 如果帧为空，跳过


        // 进行时间匹配

    while(true)
    {
        if(matched_queue.empty()) 
        {
            std::cerr<<"matched queue empty"<<std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        // 7. 在窗口中显示帧
        cv::Mat showframe = matched_queue.front().image.image;
        cv::imshow("Camera Feed", showframe);

        // 8. 等待按键事件 (等待1毫秒)
        // 这个延时对于显示视频至关重要，否则窗口会无响应
        int key = cv::waitKey(1);

        // 9. 处理按键
        if (key == ' ') { // 空格键的ASCII码是32
            // 生成文件名并拼接完整路径
            std::string filename = generate_filename(Num);
            std::string filepath = output_dir + "/" + filename;

            // 保存当前帧为PNG图片
            bool saved = cv::imwrite(filepath, showframe);
            cv::Mat R_world_to_grip(matched_queue.front().imu.orientation.toRotMat3x3());
            fs << filename << R_world_to_grip;

            
            if (saved) {
                std::cout << "图片已保存: " << filepath << std::endl;
            } else {
                std::cerr << "错误: 无法保存图片到 " << filepath << std::endl;
            }
        } else if (key == 27) { // ESC键的ASCII码是27
            std::cout << "正在退出程序..." << std::endl;
            break; // 退出循环
        }

    }
    

    fs.release();// 关闭文件
    cv::destroyAllWindows();

    return 0;
}
