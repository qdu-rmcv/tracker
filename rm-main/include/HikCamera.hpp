#ifndef IO_HIKCAMERA_HPP
#define IO_HIKCAMERA_HPP

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "thread_safe_queue.hpp"
#include <libusb-1.0/libusb.h>
#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"

namespace io{

class HikCamera
{
public:
    struct ImageData
    {
        cv::Mat image;
        std::chrono::steady_clock::time_point time;
    };
    HikCamera(unsigned int MaxframeNum,
              double exposure_ms, 
              double gain);
    
    void read(ImageData& imgdata);

    ~HikCamera();

private:

    enum class Hik {Running,Stopped};
    struct config
    {
        double exposure_ms;
        double gain;
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
    
    std::atomic<Hik> HikState;
    std::thread HikSDKthread;
    unsigned int MaxframeNum;
    tools::ThreadSafeQueue<ImageData,true> Frames{MaxframeNum};

    void ProtectRunning();

    void capture_start();
    void capture_stop();

    void set_float_value(const std::string & name, double value);
    void set_enum_value(const std::string & name, unsigned int value);

};

}
#endif 










