#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// ================= 配置 =================
const int TARGET_SIZE = 112;
// =======================================

/**
 * @brief 处理单张图片：读取 -> 缩放 -> 转三通道 -> 保存
 * * @param inputPath  原始图片路径
 * @param outputPath 处理后图片的保存路径
 */
void processAndSaveImage(const std::string &inputPath,
                         const std::string &outputPath) {
  // 1. 读取图片 (强制以灰度模式读取，应对各种奇怪的输入)
  cv::Mat src = cv::imread(inputPath, cv::IMREAD_GRAYSCALE);

  if (src.empty()) {
    std::cerr << "[错误] 无法读取图片: " << inputPath << std::endl;
    return;
  }

  cv::Mat resized_img;
  cv::Mat color_img;

  // 2. 缩放 (Resize) 到 112x112
  cv::resize(src, resized_img, cv::Size(TARGET_SIZE, TARGET_SIZE));

  // 3. 转为三通道 (Gray -> BGR)
  // 这一步只是把单通道数据复制 3 份，R=G=B
  // OpenCV 的 imwrite 默认保存 BGR 格式的图片，所以转成 BGR 即可
  cv::cvtColor(resized_img, color_img, cv::COLOR_GRAY2BGR);

  // 4. 保存图片
  // 保存后的图片就是标准的 JPG/PNG，像素值 0-255，没有归一化
  bool success = cv::imwrite(outputPath, color_img);

  if (success) {
    std::cout << "[成功] 已保存: " << outputPath << " (" << color_img.cols
              << "x" << color_img.rows << ", " << color_img.channels() << "ch)"
              << std::endl;
  } else {
    std::cerr << "[失败] 无法写入文件: " << outputPath << std::endl;
  }
}

int main() {
  // --- 示例用法 ---
  for (int i = 8001; i < 10000; i++) {
    // 假设这是你的原始图片
    std::string input_file = "/home/king/Downloads/yes/aaa/mnist/test/"+std::to_string(i)+".png";

    // 假设这是你要保存到训练集目录的位置
    std::string output_file =
        "/home/king/Pytorch/train/data/val/7_negetive/image_"+std::to_string(i+1080)+".png";

    // 执行处理
    // 实际使用时，你可以用 std::filesystem 遍历文件夹来批量调用这个函数
    processAndSaveImage(input_file, output_file);
  }
  return 0;
}