#ifndef ARMOR_DETECTOR__GPU_PREPROCESSOR_HPP_
#define ARMOR_DETECTOR__GPU_PREPROCESSOR_HPP_

#if ARMOR_DETECTOR_HAS_TENSORRT

#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>

namespace rm_auto_aim
{

// GPU 预处理: letterbox 双线性 resize + 左上对齐 padding + uint8->float +
//             /255 + HWC->CHW, 一个 kernel 融合完成.
//
// 用法 A (非 graph, raw 路径):
//     double scale = preproc.Run(rgb_img, d_input, stream);
//
// 用法 B (CUDA Graph, end2end 路径):
//     preproc.EnsureInitialized(h, w);
//     preproc.StageHost(rgb_img);          // CPU-side memcpy, 不可进 capture
//     preproc.Launch(d_input, stream);     // H2D + kernel, 可进 capture
class GpuPreprocessor
{
 public:
  struct Config
  {
    int dst_size = 640;
    bool swap_rb = false;  // 预留: BGR<->RGB 通道互换
    // 未来扩展位置: mean/std、量化 scale、最近邻/双线性开关等
  };

  explicit GpuPreprocessor(const Config& cfg);
  ~GpuPreprocessor();

  GpuPreprocessor(const GpuPreprocessor&) = delete;
  GpuPreprocessor& operator=(const GpuPreprocessor&) = delete;

  // 首次调用按 src 尺寸分配 raw buffer 并计算 letterbox 参数.
  // 之后若尺寸变化会抛异常 (CUDA Graph 要求输入尺寸稳定).
  void EnsureInitialized(int src_h, int src_w);

  // 把 cv::Mat 内容拷进 pinned host buffer (纯 CPU 操作, 不走 stream).
  // 要求 src.isContinuous() && src.type() == CV_8UC3 && 尺寸匹配.
  // 注意: 必须在 cudaStreamBeginCapture / cudaGraphLaunch 之外调用.
  void StageHost(const cv::Mat& src);

  // 发起 H2D (pinned->device) + preprocess kernel 到 stream.
  // 调用前必须已 StageHost. 可被 CUDA Graph 捕获.
  // d_dst: 目标 GPU buffer, float CHW, >= 3 * dst_size * dst_size.
  void Launch(float* d_dst, cudaStream_t stream);

  // 非 graph 场景一把梭: EnsureInitialized + StageHost + Launch (不 sync).
  // 返回 letterbox scale, 供后处理坐标反算.
  double Run(const cv::Mat& src, float* d_dst, cudaStream_t stream);

  bool IsReady() const noexcept { return ready_; }
  double GetScale() const noexcept { return scale_; }
  int GetNewH() const noexcept { return new_h_; }
  int GetNewW() const noexcept { return new_w_; }
  int GetRawH() const noexcept { return raw_h_; }
  int GetRawW() const noexcept { return raw_w_; }
  int GetDstSize() const noexcept { return cfg_.dst_size; }

 private:
  Config cfg_;
  bool ready_ = false;
  int raw_h_ = 0;
  int raw_w_ = 0;
  std::size_t raw_bytes_ = 0;
  double scale_ = 1.0;
  int new_h_ = 0;
  int new_w_ = 0;

  std::uint8_t* d_raw_ = nullptr;
  std::uint8_t* h_raw_ = nullptr;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR_HAS_TENSORRT
#endif  // ARMOR_DETECTOR__GPU_PREPROCESSOR_HPP_
