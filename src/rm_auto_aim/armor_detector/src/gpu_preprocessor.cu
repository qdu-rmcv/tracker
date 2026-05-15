#if ARMOR_DETECTOR_HAS_TENSORRT

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#include "armor_detector/gpu_preprocessor.hpp"

namespace rm_auto_aim
{
namespace
{

#define RM_CUDA_CHECK(call)                                        \
  do                                                               \
  {                                                                \
    cudaError_t err__ = (call);                                    \
    if (err__ != cudaSuccess)                                      \
    {                                                              \
      std::ostringstream oss__;                                    \
      oss__ << "CUDA error at " __FILE__ ":" << __LINE__ << " -> " \
            << cudaGetErrorString(err__);                          \
      throw std::runtime_error(oss__.str());                       \
    }                                                              \
  } while (0)

template <bool kSwapRB>
__global__ void preprocess_kernel(const std::uint8_t* __restrict__ src, int src_h,
                                  int src_w, float* __restrict__ dst, int dst_size,
                                  float inv_scale, int new_h, int new_w)
{
  const int X = blockIdx.x * blockDim.x + threadIdx.x;
  const int Y = blockIdx.y * blockDim.y + threadIdx.y;
  if (X >= dst_size || Y >= dst_size)
  {
    return;
  }

  const int PLANE_SZ = dst_size * dst_size;
  const int OUT_IDX = Y * dst_size + X;

  float c0 = NAN, c1 = NAN, c2 = NAN;

  if (X >= new_w || Y >= new_h)
  {
    // letterbox 右/下 pad 区域
    c0 = 0.0f;
    c1 = 0.0f;
    c2 = 0.0f;
  }
  else
  {
    // 中心对齐反向映射 (与 cv::resize INTER_LINEAR 对齐)
    const float FX = (X + 0.5f) * inv_scale - 0.5f;
    const float FY = (Y + 0.5f) * inv_scale - 0.5f;

    int x0 = __float2int_rd(FX);
    int y0 = __float2int_rd(FY);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    const float LX = FX - x0;
    const float LY = FY - y0;

    x0 = max(0, min(src_w - 1, x0));
    x1 = max(0, min(src_w - 1, x1));
    y0 = max(0, min(src_h - 1, y0));
    y1 = max(0, min(src_h - 1, y1));

    const int STRIDE = src_w * 3;
    const std::uint8_t* p00 =
        src + static_cast<ptrdiff_t>(y0 * STRIDE) + static_cast<ptrdiff_t>(x0 * 3);
    const std::uint8_t* p01 =
        src + static_cast<ptrdiff_t>(y0 * STRIDE) + static_cast<ptrdiff_t>(x1 * 3);
    const std::uint8_t* p10 =
        src + static_cast<ptrdiff_t>(y1 * STRIDE) + static_cast<ptrdiff_t>(x0 * 3);
    const std::uint8_t* p11 =
        src + static_cast<ptrdiff_t>(y1 * STRIDE) + static_cast<ptrdiff_t>(x1 * 3);

    const float W00 = (1.f - LY) * (1.f - LX);
    const float W01 = (1.f - LY) * LX;
    const float W10 = LY * (1.f - LX);
    const float W11 = LY * LX;

    const float V0 = W00 * p00[0] + W01 * p01[0] + W10 * p10[0] + W11 * p11[0];
    const float V1 = W00 * p00[1] + W01 * p01[1] + W10 * p10[1] + W11 * p11[1];
    const float V2 = W00 * p00[2] + W01 * p01[2] + W10 * p10[2] + W11 * p11[2];

    if constexpr (kSwapRB)
    {
      c0 = V2;
      c1 = V1;
      c2 = V0;
    }
    else
    {
      c0 = V0;
      c1 = V1;
      c2 = V2;
    }

    constexpr float K_INV255 = 1.0f / 255.0f;
    c0 *= K_INV255;
    c1 *= K_INV255;
    c2 *= K_INV255;
  }

  dst[0 * PLANE_SZ + OUT_IDX] = c0;
  dst[1 * PLANE_SZ + OUT_IDX] = c1;
  dst[2 * PLANE_SZ + OUT_IDX] = c2;
}

}  // namespace

GpuPreprocessor::GpuPreprocessor(const Config& cfg) : cfg_(cfg)
{
  if (cfg_.dst_size <= 0)
  {
    throw std::runtime_error("GpuPreprocessor: dst_size must be > 0");
  }
}

GpuPreprocessor::~GpuPreprocessor()
{
  if (d_raw_)
  {
    cudaFree(d_raw_);
  }
  if (h_raw_)
  {
    cudaFreeHost(h_raw_);
  }
}

void GpuPreprocessor::EnsureInitialized(int src_h, int src_w)
{
  if (ready_)
  {
    if (src_h != raw_h_ || src_w != raw_w_)
    {
      throw std::runtime_error(
          "GpuPreprocessor: input size changed at runtime (initial " +
          std::to_string(raw_h_) + "x" + std::to_string(raw_w_) + ", got " +
          std::to_string(src_h) + "x" + std::to_string(src_w) + ")");
    }
    return;
  }

  if (src_h <= 0 || src_w <= 0)
  {
    throw std::runtime_error("GpuPreprocessor: invalid src size");
  }

  raw_h_ = src_h;
  raw_w_ = src_w;
  raw_bytes_ = static_cast<std::size_t>(src_h) * src_w * 3;

  RM_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_raw_), raw_bytes_));
  RM_CUDA_CHECK(
      cudaHostAlloc(reinterpret_cast<void**>(&h_raw_), raw_bytes_, cudaHostAllocDefault));

  // 与 YoloDetector 原 CPU 版 letterbox 计算保持一致
  const double SX = static_cast<double>(cfg_.dst_size) / src_h;
  const double SY = static_cast<double>(cfg_.dst_size) / src_w;
  scale_ = std::min(SX, SY);
  new_h_ = static_cast<int>(src_h * scale_);
  new_w_ = static_cast<int>(src_w * scale_);

  ready_ = true;
}

void GpuPreprocessor::StageHost(const cv::Mat& src)
{
  if (!ready_)
  {
    throw std::runtime_error("GpuPreprocessor: StageHost before EnsureInitialized");
  }
  if (src.rows != raw_h_ || src.cols != raw_w_)
  {
    throw std::runtime_error("GpuPreprocessor: src size mismatch in StageHost");
  }
  if (!src.isContinuous())
  {
    throw std::runtime_error("GpuPreprocessor: src must be continuous");
  }
  if (src.type() != CV_8UC3)
  {
    throw std::runtime_error("GpuPreprocessor: src must be CV_8UC3");
  }

  std::memcpy(h_raw_, src.data, raw_bytes_);
}

void GpuPreprocessor::Launch(float* d_dst, cudaStream_t stream)
{
  if (!ready_)
  {
    throw std::runtime_error("GpuPreprocessor: Launch before EnsureInitialized");
  }

  RM_CUDA_CHECK(
      cudaMemcpyAsync(d_raw_, h_raw_, raw_bytes_, cudaMemcpyHostToDevice, stream));

  const dim3 BLOCK(16, 16);
  const dim3 GRID((cfg_.dst_size + BLOCK.x - 1) / BLOCK.x,
                  (cfg_.dst_size + BLOCK.y - 1) / BLOCK.y);
  const float INV_SCALE = static_cast<float>(1.0 / scale_);

  if (cfg_.swap_rb)
  {
    preprocess_kernel<true><<<GRID, BLOCK, 0, stream>>>(
        d_raw_, raw_h_, raw_w_, d_dst, cfg_.dst_size, INV_SCALE, new_h_, new_w_);
  }
  else
  {
    preprocess_kernel<false><<<GRID, BLOCK, 0, stream>>>(
        d_raw_, raw_h_, raw_w_, d_dst, cfg_.dst_size, INV_SCALE, new_h_, new_w_);
  }
}

double GpuPreprocessor::Run(const cv::Mat& src, float* d_dst, cudaStream_t stream)
{
  EnsureInitialized(src.rows, src.cols);
  StageHost(src);
  Launch(d_dst, stream);
  return scale_;
}

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR_HAS_TENSORRT
