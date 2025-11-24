#include "../include/NumClassifier.hpp"
#include <opencv2/core/types.hpp>
#include <vector>


NumClassifier::NumClassifier(std::string model_path)
{
    Net = cv::dnn::readNetFromONNX(model_path);
    // 设置首选的计算后端为 OpenVINO Inference Engine
    // Net.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
    // 设置首选的计算目标设备为 CPU
    Net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    // Create blob from image

}
std::vector<NumClassifier::Ans> NumClassifier::Classify(const std::vector<cv::Mat>& armors_pattern)
{
    std::vector<NumClassifier::Ans> ans;
    if(armors_pattern.empty()) return ans;
    ans.reserve(armors_pattern.size());

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImages(armors_pattern, blob, 0.01735, cv::Size(112, 112),cv::Scalar(123.675, 116.28, 103.53), true, false);

    // Set the input blob for the neural network
    this->Net.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = this->Net.forward();

    auto Softmax = [](cv::Mat& output)-> cv::Point 
    {
        // 1. 找到最大值
        double minVal, maxVal;
        cv::Point maxPosi;
        minMaxLoc(output, 0, &maxVal, 0, &maxPosi);

        // 2. 减去最大值 (防止 exp 溢出)
        output = output - maxVal;

        // 3. 计算指数
        cv::exp(output, output);

        // 4. 计算和
        float sum = cv::sum(output)[0];

        // 5. 归一化
        output /= sum;
        return maxPosi;
    };

    //读取结果
    for (int i = 0; i < outputs.rows; ++i) 
    {
        // 获取第 i 张图片对应的得分行
        cv::Mat scores = outputs.row(i);
        auto Poi = Softmax(scores);

        ans.emplace_back(Poi.x,scores.at<float>(Poi.x));
    }
    return ans;
}
