#include "../include/HikCamera.hpp"
#include "../include/logger.hpp"
#include <chrono>
#include <ratio>
#include <thread>

namespace io
{
HikCamera::HikCamera(double exposure_ms, 
                     double gain,
                     bool autocap)
                     :HikState(Hik::Stopped)
{

    this->parame.exposure_ms = exposure_ms*1e3;
    this->parame.gain = gain;
    this->parame.autocap = autocap;

    this->capture_init();
    this->ProtectRunning();
}

HikCamera::~HikCamera()
{
  if (this->guard.protectthread.joinable()) this->guard.protectthread.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikCamera::read(ImageData& imgdata)
{
  if(conCapOpen)
  {
    this->Frames.pop(imgdata);
    return;
  }
  
  if(this->HikState == Hik::Stopped) return;
  
  MV_FRAME_OUT raw;
  unsigned int ret;
  unsigned int nMsec = 100;

  ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
  }

  auto timestamp = std::chrono::steady_clock::now();
  cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

  const auto & frame_info = raw.stFrameInfo;
  auto pixel_type = frame_info.enPixelType;
  cv::Mat dst_image;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
    {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
    {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
    {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
    {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
  cv::cvtColor(img, dst_image, type_map.at(pixel_type));
  img = dst_image;

  ret = MV_CC_FreeImageBuffer(handle_, &raw);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
  }

  imgdata = {img, timestamp};
}


void HikCamera::capture_init()
{
  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  unsigned int nImageNodeNum = 1;
  ret = MV_CC_SetImageNodeNum(handle_, nImageNodeNum);
  if (MV_OK != ret)
  {
      // 设置失败
      tools::logger()->warn("MV_CC_SetImageNodeNum failed: {:#x}", ret);
      return;
  }

  if(!this->parame.autocap)
  {
    ret = MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous");
    if (MV_OK != ret) {
        tools::logger()->warn("Set Acquisition Mode to Continuous fail! nRet [0x%x]\n", ret);
        return;
    }
  
    //    将触发模式设置为开启 (On)
    //    参数 "TriggerMode" 的值: 0 表示 Off, 1 表示 On
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 1); 
    if (MV_OK != ret) {
        tools::logger()->warn("Set Trigger Mode to On fail! nRet [0x%x]\n", ret);
        return;
    }
  
    //    设置触发源为外部硬件触发 (Line0)
    //    可用的值通常有 "Line0", "Line1", "Line2", "Software", "FrequencyConverter" 等
    //    请根据您的物理接线选择正确的一项
    ret = MV_CC_SetEnumValueByString(handle_, "TriggerSource", "Line0");
    if (MV_OK != ret) {
        tools::logger()->warn("Set Trigger Source to Line0 fail! nRet [0x%x]\n", ret);
        return;
    }
  
    //    (可选) 设置触发激活方式
    //    例如设置为上升沿触发 "RisingEdge"
    //    其他可选值如 "FallingEdge", "LevelHigh", "LevelLow"
    ret = MV_CC_SetEnumValueByString(handle_, "TriggerActivation", "RisingEdge");
    if (MV_OK != ret) {
        tools::logger()->warn("Set Trigger Activation to RisingEdge fail! nRet [0x%x]\n", ret);
        return;
    }
  }else{
    //将触发模式设置为开启 (On)
    //参数 "TriggerMode" 的值: 0 表示 Off, 1 表示 On
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0); 
    if (MV_OK != ret) {
        tools::logger()->warn("Set Trigger Mode to Off fail! nRet [0x%x]\n", ret);
        return;
    }
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", this->parame.exposure_ms);
  set_float_value("Gain", this->parame.gain);


  ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", 249.0);;
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(set framerate) failed: {:#x}", ret);
    return;
  }
  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  this->HikState = Hik::Running;
}

void HikCamera::continueCap(size_t MaxframeNum)
{
  this->conCapOpen =true;
  this->MaxframeNum = MaxframeNum;
  this->Frames.setSize(this->MaxframeNum);

  if(this->HikState == Hik::Stopped) return;

  this->HikSDKthread = std::thread{[this] {

    tools::logger()->info("HikRobot's capture thread started."); 
    MV_FRAME_OUT raw;

    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    while (true) {
      // std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      cv::Mat dst_image;
      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
      cv::cvtColor(img, dst_image, type_map.at(pixel_type));
      img = dst_image;

      Frames.push({img, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    std::unique_lock<std::mutex> lock(this->guard.mux);
    this->HikState = Hik::Stopped;
    this->guard.HikIsquit.notify_all();
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikCamera::ProtectRunning()
{
  this->guard.protectthread = std::thread
  {
    [this]()->void
    {
      std::unique_lock<std::mutex> lock(this->guard.mux);
      while(true)
      {
        this->guard.HikIsquit.wait(lock,[this] { return (this->HikState == Hik::Stopped);});
        this->capture_stop();
        this->capture_init();
        if(conCapOpen) this->continueCap(this->MaxframeNum);
      }
    }
  };
}

void HikCamera::capture_stop()
{
  this->HikState = Hik::Stopped;
  if (HikSDKthread.joinable()) HikSDKthread.join();

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_SetCommandValue(handle_, "DeviceReset");
  if (ret != MV_OK) {
      tools::logger()->error("Hard Reset failed: MV_CC_SetCommandValue('DeviceReset') failed with {:#x}", ret);
      // 即使失败，也尝试关闭设备
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      return ;
  }
  
  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }
}


void HikCamera::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikCamera::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

}



