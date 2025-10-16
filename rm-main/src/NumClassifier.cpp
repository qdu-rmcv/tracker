#include "../include/NumClassifier.hpp"
#include <vector>


NumClassifier::NumClassifier(std::string model_path)
{
    Net = cv::dnn::readNetFromONNX(model_path);
    // 设置首选的计算后端为 OpenVINO Inference Engine
    Net.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
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
    cv::dnn::blobFromImages(armors_pattern, blob);

     // Set the input blob for the neural network
    this->Net.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = this->Net.forward();

    //读取结果
    for (int i = 0; i < outputs.rows; ++i) 
    {
        // 获取第 i 张图片对应的得分行
        cv::Mat scores = outputs.row(i);

        // Do softmax
        float max_prob = *std::max_element(scores.begin<float>(), scores.end<float>()); //max_element函数返回的是一个迭代器，要获取实际值，需要解引用*
        cv::Mat softmax_prob;
        cv::exp(scores - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;
        ans.emplace_back(label_id,confidence);
    }
    return ans;
}
