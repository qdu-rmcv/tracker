#ifndef IO_HIKCAMERA_HPP
#define IO_HIKCAMERA_HPP

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <condition_variable>
#include <chrono>

#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include "thread_safe_queue.hpp"

namespace io{
using namespace std::chrono_literals;

class HikCamera
{
public:
    
    struct ImageData
    {
        cv::Mat image;
        std::chrono::steady_clock::time_point time;
    };
    HikCamera(double exposure_ms, 
              double gain,
              bool autocap=true);

    void read(ImageData& imgdata);
    void continueCap(size_t MaxframeNum);

    ~HikCamera();

private:

    enum class Hik {Running,Stopped};
    struct config
    {
        double exposure_ms;
        double gain;
        bool autocap;
    };
    struct protect
    {

        std::mutex mux;
        std::condition_variable HikIsquit;
        std::thread protectthread;    
    };
    
    
    int vid_, pid_;
    void * handle_;
    
    config parame;
    protect guard;

    bool conCapOpen = false;
    
    std::atomic<Hik> HikState;
    std::thread HikSDKthread;
    size_t MaxframeNum=0;
    tools::ThreadSafeQueue<ImageData,true> Frames{0};

    void ProtectRunning();

    void capture_init();

    void capture_stop();

    void set_float_value(const std::string & name, double value);
    void set_enum_value(const std::string & name, unsigned int value);

};

}
#endif 










