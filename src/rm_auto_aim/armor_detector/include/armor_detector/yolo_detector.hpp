#ifndef ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
#define ARMOR_DETECTOR__YOLO_DETECTOR_HPP_

#if ARMOR_DETECTOR_HAS_OPENVINO || ARMOR_DETECTOR_HAS_TENSORRT

#include <cstddef>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_base.hpp"

#if ARMOR_DETECTOR_HAS_TENSORRT
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime.h>

#include "armor_detector/gpu_preprocessor.hpp"

#elif ARMOR_DETECTOR_HAS_OPENVINO
#include <openvino/openvino.hpp>
#endif  // ARMOR_DETECTOR_HAS_OPENVINO / ARMOR_DETECTOR_HAS_TENSORRT

namespace rclcpp
{
class Node;
}

namespace rm_auto_aim
{

class YoloDetector : public DetectorBase
{
 public:
  struct YoloParams
  {
    std::string model_path;
    std::string device = "CPU";
    int input_size = 640;
    float score_threshold = 0.7f;
    float min_confidence = 0.8f;
    float nms_threshold = 0.3f;
    std::vector<std::string> ignore_classes;
    int detect_color;
    int num_keypoints = 4;
    float large_armor_ratio_threshold = 3.2f;
    bool end_to_end = false;
  };

  static std::unique_ptr<YoloDetector> Create(rclcpp::Node& node);

  explicit YoloDetector(const YoloParams& params);

  ~YoloDetector() override;
  YoloDetector(const YoloDetector&) = delete;
  YoloDetector& operator=(const YoloDetector&) = delete;

  DetectionResult Detect(const cv::Mat& rgb_img) override;

  void DrawResults(cv::Mat& img) override;

 private:
  std::vector<Armor> Parse(double scale, const cv::Mat& output);
#if ARMOR_DETECTOR_HAS_TENSORRT
  std::vector<Armor> ParseEnd2End(double scale);

  void InitTrtRaw();
  void InitTrtEnd2End();
  DetectionResult DetectTrtRaw(const cv::Mat& rgb_img);
  DetectionResult DetectTrtEnd2End(const cv::Mat& rgb_img);
#elif ARMOR_DETECTOR_HAS_OPENVINO
  std::vector<Armor> ParseOpenVinoEnd2End(double scale, int n, const float* scores,
                                          const int* classes, const float* kpts,
                                          int kpt_channels);

  // 根据输入图像分辨率刷新 letterbox 比例缓存与持久 input buffer 的 padding 区。
  // 仅当输入分辨率变化时才会执行实际工作。
  void RefreshLetterboxCache(int rows, int cols);
#endif
  void SortKeypoints(std::vector<cv::Point2f>& keypoints);
  ArmorType DetermineArmorType(const Light& light_1, const Light& light_2);

  // 预计算的 per-class LUT, 避免每帧字符串比较 / map_label / std::find。
  // 由构造函数在配置完 ignore_classes 后一次性建立, 之后只读。
  std::vector<std::string> class_label_lut_;     // raw_label → mapped label
  std::vector<int> class_color_lut_;             // RED / BLUE / -1 (颜色无关)
  std::vector<std::uint8_t> class_ignored_lut_;  // 1=该 class 命中 ignore_classes

  YoloParams params_;
  int class_num_;

#if ARMOR_DETECTOR_HAS_TENSORRT
  std::unique_ptr<nvinfer1::IRuntime> trt_runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> trt_engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> trt_context_;

  std::string trt_input_name_;
  std::string trt_output_name_;

  nvinfer1::Dims trt_input_dims_{};
  nvinfer1::Dims trt_output_dims_{};

  void* trt_d_input_ = nullptr;
  void* trt_d_output_ = nullptr;
  cudaStream_t trt_stream_ = nullptr;

  std::size_t trt_input_bytes_ = 0;
  std::size_t trt_output_bytes_ = 0;

  std::vector<float> trt_host_output_;

  // IO 张量名
  std::string in_name_;           // images / input
  std::string out_num_name_;      // num_dets     [1, 1]        int32
  std::string out_boxes_name_;    // det_boxes    [1, K, 4]     fp32  (xyxy, 640 尺度)
  std::string out_scores_name_;   // det_scores   [1, K]        fp32
  std::string out_classes_name_;  // det_classes  [1, K]        int32
  std::string out_kpts_name_;     // det_kpts     [1, K, K*D]   fp32  (640 尺度)

  // Device / pinned host buffers
  void* d_input_ = nullptr;
  int* d_num_ = nullptr;
  float* d_boxes_ = nullptr;
  float* d_scores_ = nullptr;
  int* d_classes_ = nullptr;
  float* d_kpts_ = nullptr;

  int* h_num_ = nullptr;
  float* h_boxes_ = nullptr;
  float* h_scores_ = nullptr;
  int* h_classes_ = nullptr;
  float* h_kpts_ = nullptr;

  cudaStream_t stream_ = nullptr;

  // 形状元数据
  int keep_topk_ = 0;     // engine 里 NMS 的 max_output_boxes
  int kpt_channels_ = 0;  // num_kpts * kp_dim (2 或 3)
  std::size_t input_bytes_ = 0;

  // CUDA Graph
  cudaGraph_t graph_ = nullptr;
  cudaGraphExec_t graph_exec_ = nullptr;
  bool graph_ready_ = false;

  std::unique_ptr<GpuPreprocessor> preprocessor_;

  // cudaEvent_t ev_start_ = nullptr;
  // cudaEvent_t ev_end_ = nullptr;

#elif ARMOR_DETECTOR_HAS_OPENVINO
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;

  // 持久化 input buffer。直接 wrap OpenVINO infer_request 的内置 input tensor,
  // 避免每帧 cv::Mat 分配 + cv::Scalar 全图清零 + ov::Tensor 构造 + set_input_tensor。
  ov::Tensor ov_input_tensor_;
  cv::Mat ov_input_mat_;

  // letterbox 比例缓存。相机分辨率不变时 (实际场景 99% 都是) 这些值整轮复用。
  int ov_last_rows_ = 0;
  int ov_last_cols_ = 0;
  double ov_cached_scale_ = 0.0;
  int ov_cached_w_ = 0;
  int ov_cached_h_ = 0;
#endif  // ARMOR_DETECTOR_HAS_OPENVINO / ARMOR_DETECTOR_HAS_TENSORRT

  std::vector<Armor> last_armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR_HAS_OPENVINO || ARMOR_DETECTOR_HAS_TENSORRT

#endif  // ARMOR_DETECTOR__YOLO_DETECTOR_HPP_