#include "HikCamera.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <iomanip>    // 用于格式化时间字符串
#include <sstream>    // 用于构建字符串

// 函数：生成一个基于当前时间的唯一文件名
static int Num=0;

std::string generate_filename(int &Num) {
    
    std::string ss;
    ss ="image_"+std::to_string(Num++)+"_.png";
    return ss;
}

int main() {
    // 1. 定义数据保存目录
    std::string output_dir = "../../CameraCalibration/CalibrationImage";



    // 3. 打开默认摄像头 (ID 为 0)
    io::HikCamera Hik(1,10,0);
    io::HikCamera::ImageData frames; 

    // 4. 检查摄像头是否成功打开


    std::cout << "摄像头已成功打开。按 '空格键' 截图，按 'ESC' 退出。" << std::endl;

    // 5. 创建一个窗口用于显示
    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);


    while (true) {
        // 6. 从摄像头捕获一帧
        Hik.read(frames);
        cv::Mat frame = frames.image;

        // 如果帧为空，表示视频流结束或摄像头断开


        // 7. 在窗口中显示帧
        cv::imshow("Camera Feed", frame);

        // 8. 等待按键事件 (等待1毫秒)
        // 这个延时对于显示视频至关重要，否则窗口会无响应
        int key = cv::waitKey(1);

        // 9. 处理按键
        if (key == ' ') { // 空格键的ASCII码是32
            // 生成文件名并拼接完整路径
            std::string filename = generate_filename(Num);
            std::string filepath = output_dir + "/" + filename;

            // 保存当前帧为PNG图片
            bool saved = cv::imwrite(filepath, frame);
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

    // 10. 释放资源

    cv::destroyAllWindows();

    return 0;
}
