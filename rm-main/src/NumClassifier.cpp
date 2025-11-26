#include "../include/NumClassifier.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

// ==========================================
// 【性能核心】固定最大 Batch Size
// 告诉 GPU 永远只申请 8 张图的显存，
// 这样就再也不用调用 set_shape 了，速度从 1s 优化到 1ms。
// ==========================================
static const size_t MAX_BATCH_SIZE = 3;

NumClassifier::NumClassifier(std::string model_path)
{
    // 1. 读取模型
    std::shared_ptr<ov::Model> model = core.read_model(model_path);

    // 2. 配置预处理 (PrePostProcessor)
    // 这一步相当于把 blobFromImages 搬到了 GPU 内部执行
    ov::preprocess::PrePostProcessor ppp(model);
    
    // [输入定义] 告诉 OpenVINO 我们传进来的是什么：
    // 1. u8: 原始像素 0-255
    // 2. NHWC: OpenCV Mat 的默认内存布局 [Batch, Height, Width, Channel]
    // 3. BGR: OpenCV 默认颜色顺序
    ppp.input().tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC") 
        .set_color_format(ov::preprocess::ColorFormat::BGR);

    // [预处理步骤]
    // 这些步骤会自动在 GPU 上执行
    ppp.input().preprocess()
        .convert_color(ov::preprocess::ColorFormat::RGB) // BGR 转 RGB (匹配 PyTorch)
        .convert_element_type(ov::element::f16)          // 转 float
        .mean({123.675f, 116.280f, 103.530f})            // ImageNet Mean * 255
        // 【精度核心】这里必须使用 PyTorch ImageNet 的精确 Std * 255
        // R: 0.229*255=58.395, G: 0.224*255=57.12, B: 0.225*255=57.375
        .scale({58.395f, 57.120f, 57.375f});

    // 模型内部原本需要的输入布局 (通常 ONNX 导出是 NCHW)
    ppp.input().model().set_layout("NCHW");

    // 构建模型，将预处理步骤融入模型图层中
    model = ppp.build();

    // ==========================================================
    // 【维度陷阱修复】Reshape 的维度必须匹配 "NHWC" 布局！
    // 之前错误写法: {8, 3, 112, 112} -> 导致 H=3, C=112 (图像错乱/识别为负样本)
    // 现在正确写法: {8, 112, 112, 3} -> 对应 N, H, W, C
    // ==========================================================
    std::map<std::string, ov::PartialShape> shapes;
    shapes[model->input().get_any_name()] = ov::PartialShape{MAX_BATCH_SIZE, 112, 112, 3}; 
    model->reshape(shapes);

    // 3. 编译模型
    bool gpu_success = false;
    try {
        std::cout << "[OpenVINO] 正在加载到 GPU (FP32 + NHWC Reshape)..." << std::endl;
        
        ov::AnyMap gpu_props;
        // 使用 FP32 保证 "3 vs 7" 的识别精度 (小模型上 FP32 往往比 FP16 更快)
        gpu_props[ov::hint::inference_precision.name()] = ov::element::f32; 
        // 开启低延迟模式
        gpu_props[ov::hint::performance_mode.name()] = ov::hint::PerformanceMode::LATENCY;
        // 开启模型缓存，第二次启动秒开
        gpu_props[ov::cache_dir.name()] = "./gpu_cache";

        compiled_model = core.compile_model(model, "GPU", gpu_props);
        gpu_success = true;
        std::cout << "[OpenVINO] 成功: 模型已加载到核显。" << std::endl;
    } 
    catch (const std::exception& e) {
        std::cerr << "[OpenVINO] 警告: 核显加载失败: " << e.what() << std::endl;
    }

    // 4. 回退 CPU (如果 GPU 挂了)
    if (!gpu_success) {
        try {
            std::cout << "[OpenVINO] 正在回退到 CPU 模式..." << std::endl;
            ov::AnyMap cpu_props;
            cpu_props[ov::hint::performance_mode.name()] = ov::hint::PerformanceMode::LATENCY;
            compiled_model = core.compile_model(model, "CPU", cpu_props);
        } catch (const std::exception& ex) {
            std::cerr << "[OpenVINO] 致命错误: CPU 也无法加载模型！" << ex.what() << std::endl;
            exit(-1); 
        }
    }

    // 5. 创建推理请求
    infer_request = compiled_model.create_infer_request();
    
    // 6. 热身 (Warm-up)
    // 跑一次空数据，消除 GPU 首帧编译带来的延迟
    try {
        ov::Tensor input_tensor = infer_request.get_input_tensor();
        std::memset(input_tensor.data(), 0, input_tensor.get_byte_size());
        infer_request.infer();
    } catch (...) {}
}

std::vector<NumClassifier::Ans> NumClassifier::Classify(const std::vector<cv::Mat>& armors_pattern)
{
    std::vector<NumClassifier::Ans> ans;
    if(armors_pattern.empty()) return ans;

    // 1. 获取输入 Tensor (显存映射内存)
    // 注意：这里的 input_tensor 形状已经是 [8, 112, 112, 3] (NHWC)
    ov::Tensor input_tensor = infer_request.get_input_tensor();
    uint8_t* input_data_ptr = input_tensor.data<uint8_t>();
    
    // 计算单张图片的大小 (112 * 112 * 3 字节)
    size_t img_pixels = 112 * 112 * 3;
    size_t valid_count = std::min(armors_pattern.size(), MAX_BATCH_SIZE);

    // 2. 数据填充 (Padding 策略)
    for (size_t i = 0; i < MAX_BATCH_SIZE; ++i) {
        if (i < valid_count) {
            cv::Mat resized_img;
            // 确保输入尺寸严格为 112x112
            if (armors_pattern[i].cols != 112 || armors_pattern[i].rows != 112) {
                // 使用线性插值，匹配 PyTorch Resize 行为
                cv::resize(armors_pattern[i], resized_img, cv::Size(112, 112), 0, 0, cv::INTER_LINEAR);
            } else {
                resized_img = armors_pattern[i];
            }
            // 直接拷贝原始像素 (uint8)，不做任何除法！
            std::memcpy(input_data_ptr + i * img_pixels, resized_img.data, img_pixels);
        } 
        // 超过有效数量的部分不需要处理，留着旧数据也没关系，反正不读结果
    }

    // 3. 执行推理
    // 没有任何 set_shape 调用，OpenVINO 直接复用之前的图，耗时 < 2ms
    infer_request.infer();

    // 4. 获取输出
    const ov::Tensor& output_tensor = infer_request.get_output_tensor();
    const float* raw_output = output_tensor.data<float>();
    
    ov::Shape out_shape = output_tensor.get_shape();
    // out_shape 是 [8, ClassNum]
    int class_num = out_shape[1]; 

    // 5. 后处理 (只读取前 valid_count 个结果)
    auto Softmax = [](cv::Mat& output)-> cv::Point 
    {
        double minVal, maxVal;
        cv::Point maxPosi;
        cv::minMaxLoc(output, 0, &maxVal, 0, &maxPosi);
        output = output - maxVal;
        cv::exp(output, output);
        float sum = cv::sum(output)[0];
        output /= sum;
        return maxPosi;
    };

    for (size_t i = 0; i < valid_count; ++i) 
    {
        // 手动定位到第 i 行结果的起始位置
        const float* row_ptr = raw_output + i * class_num;
        // 零拷贝包装成 cv::Mat
        cv::Mat scores(1, class_num, CV_32F, (void*)row_ptr);
        
        auto Poi = Softmax(scores);
        ans.emplace_back(Poi.x, scores.at<float>(Poi.x));
    }

    return ans;
}