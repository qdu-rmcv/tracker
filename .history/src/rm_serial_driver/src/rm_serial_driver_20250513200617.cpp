// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <buff_interfaces/msg/detail/buff_send__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  armor_or_buff_ = this->declare_parameter("armor_or_buff", 0);

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_mode", 10);
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  velocity_pub_ = this->create_publisher<auto_aim_interfaces::msg::Velocity>("/current_velocity", 10);
  buff_velocity_pub_ = this->create_publisher<buff_interfaces::msg::Velocity>("/tracker/current_velocity", 10);

  aim_time_info_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::TimeInfo>("/time_info/aim", 10);
  buff_time_info_pub_ =
    this->create_publisher<buff_interfaces::msg::TimeInfo>("/time_info/buff", 10);

  record_controller_pub_ = this->create_publisher<std_msgs::msg::String>("/record_controller", 10);

  receive_pub_ = this->create_publisher<auto_aim_interfaces::msg::Receive>("/tracker/receive", 10);

  all_latency_pub_ = this->create_publisher<auto_aim_interfaces::msg::AllLatency>("/allLatency", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  // Target change service cilent
  change_target_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/change");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // aiming_point_.header.frame_id = "odom";
  // aiming_point_.ns = "aiming_point";
  // aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  // aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  // aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  // aiming_point_.color.r = 1.0;
  // aiming_point_.color.g = 1.0;
  // aiming_point_.color.b = 1.0;
  // aiming_point_.color.a = 1.0;
  // aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  // aim_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "/tracker/send", rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  armor_send_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
    "/tracker/send", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendArmorData, this, std::placeholders::_1));

  buff_send_sub_ = this->create_subscription<buff_interfaces::msg::BuffSend>(
    "/tracker/buffsend", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendBuffData, this, std::placeholders::_1));

    // RCLCPP_ERROR(get_logger(), "模式为:%d",ArmorOrBuff);

  // // 订阅目标数据与时间信息
  aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  aim_time_info_sub_.subscribe(this, "/time_info/aim");

  // aim_sync_ = std::make_unique<AimSync>(aim_syncpolicy(500), aim_sub_, aim_time_info_sub_);
  // aim_sync_->registerCallback(
  //   std::bind(&RMSerialDriver::sendArmorData, this, std::placeholders::_1, std::placeholders::_2));

  // // 不再同步 rune + buff
  // rune_sub_.subscribe(this, "/tracker/rune");
  // buff_time_info_sub_.subscribe(this, "/time_info/buff");

  // buff_sync_ = std::make_unique<BuffSync>(buff_syncpolicy(1500), rune_sub_, buff_time_info_sub_);
  // buff_sync_->registerCallback(
  //   std::bind(&RMSerialDriver::sendBuffData, this, std::placeholders::_1, std::placeholders::_2));

}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        // crc 部分
        bool crc_ok =
          // crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
          crc16::CRC16_Verify(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)); 
        if (crc_ok) {
          
          packet.detect_color = 0;
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }


          // RCLCPP_WARN(get_logger(), "[Receive] 1 ");

          if (packet.reset_tracker) {
            resetTracker();
          }

          // 新东西
          // if (packet.change_target) {
          //   changeTarget();
          // }
          if (0) {
            changeTarget();
          }
          // RCLCPP_WARN(get_logger(), "[Receive] 2 ");

          // 将 电控来的 [0~2PI] -> [-PI ~ PI]
          packet.pitch = RMSerialDriver::angle_re_trans(packet.pitch); 
          packet.yaw = RMSerialDriver::angle_re_trans(packet.yaw); 
          packet.roll = RMSerialDriver::angle_re_trans(packet.roll); 

          // RCLCPP_WARN(get_logger(), "[Receive] 3 ");

          // RCLCPP_WARN(get_logger(), "[Receive] yaw %f", packet.yaw);
          std_msgs::msg::String task;
          std::string theory_task;

          theory_task = "aim";
          // RCLCPP_WARN(get_logger(), "[Receive] 4 ");

          // if (
          //   (packet.game_time >= 329 && packet.game_time <= 359) ||
          //   (packet.game_time >= 239 && packet.game_time <= 269)) {
          //   theory_task = "small_buff";
          // } else if (
          //   (packet.game_time >= 149 && packet.game_time <= 179) ||
          //   (packet.game_time >= 74 && packet.game_time <= 104) ||
          //   (packet.game_time > 0 && packet.game_time <= 29)) {
          //   theory_task = "large_buff";
          // } else {
          //   theory_task = "aim";
          // }

          // if (packet.task_mode == 0) {
          //   task.data = theory_task;
          // } else if (packet.task_mode == 1) {
          //   task.data = "aim";
          // } else if (packet.task_mode == 2) {
          //   if (theory_task == "aim") {
          //     task.data = "auto";
          //   } else {
          //     task.data = theory_task;
          //   }
          // } else {
          //   task.data = "aim";
          // }
          
          if (armor_or_buff_ == 1){
            task.data = "aim";
          } else if(armor_or_buff_ == 0 ){
            task.data = "large_buff";
          }
          task_pub_->publish(task);

          // RCLCPP_WARN(get_logger(), "[Receive] 5 ");


          // RCLCPP_DEBUG(
          //   get_logger(), "Game time: %d, Task mode: %d, Theory task: %s", packet.game_time,
          //   packet.task_mode, theory_task.c_str());

          std_msgs::msg::String record_controller;

          

          // record_controller.data = packet.is_play ? "start" : "stop";
          record_controller.data = "stop";

          auto_aim_interfaces::msg::Receive receive_now;

          receive_now.header.stamp = this->now();
          receive_now.pitch = packet.pitch;
          receive_now.yaw = packet.yaw;
          receive_now.roll = packet.roll;

          receive_pub_ -> publish(receive_now);

          record_controller_pub_->publish(record_controller);
//
          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          tf2::Quaternion q;
          q.setRPY(packet.roll, -packet.pitch, packet.yaw);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

          // RCLCPP_WARN(get_logger(), "[Receive] 7 ");

          // // publish time

          rclcpp::Time time_info = this->get_clock()->now();  // 在 Node 类中


          // auto_aim_interfaces::msg::TimeInfo aim_time_info;
          // buff_interfaces::msg::TimeInfo buff_time_info;
          // aim_time_info.header = t.header;
          // aim_time_info.time = packet.timestamp;
          // buff_time_info.header = t.header;
          // buff_time_info.time = packet.timestamp;
          // aim_time_info_pub_->publish(aim_time_info);
          // buff_time_info_pub_->publish(buff_time_info);
//

          // 速度发布
          auto_aim_interfaces::msg::Velocity current_velocity;
          buff_interfaces::msg::Velocity buff_current_velocity;

          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();

          current_velocity.header.stamp =
            this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          current_velocity.velocity = packet.current_v;

          buff_current_velocity.header.stamp = current_velocity.header.stamp;
          buff_current_velocity.velocity = current_velocity.velocity;
          
          velocity_pub_->publish(current_velocity);
          buff_velocity_pub_->publish(buff_current_velocity);

          // if (abs(packet.aim_x) > 0.01) {
          //   aiming_point_.header.stamp = this->now();
          //   aiming_point_.pose.position.x = packet.aim_x;
          //   aiming_point_.pose.position.y = packet.aim_y;
          //   aiming_point_.pose.position.z = packet.aim_z;
          //   marker_pub_->publish(aiming_point_);
          // }
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

// 发送装甲板数据
void RMSerialDriver::sendArmorData(const auto_aim_interfaces::msg::Send::SharedPtr armor_msg)
{
  // const static std::map<std::string, uint8_t> id_unit8_map{
  //   {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
  //   {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    float pitch;
    float yaw;

    packet.is_fire = armor_msg->is_fire;
    packet.x = armor_msg->position.x;
    packet.y = armor_msg->position.y;
    packet.z = armor_msg->position.z;
    packet.v_yaw = armor_msg->v_yaw;

    pitch = armor_msg->pitch;
    yaw = armor_msg->yaw;

    // RCLCPP_ERROR(get_logger(), "sendarmor.yaw:%f", yaw);

    // 关于 pitch 硬补;
    packet.pitch = RMSerialDriver::angle_trans(pitch);
    packet.yaw = RMSerialDriver::angle_trans(yaw);


    // SendPacket packet;
    // packet.state = msg->tracking ? 1 : 0;
    // packet.id = id_unit8_map.at(msg->id);
    // packet.armors_num = msg->armors_num;
    // packet.x = msg->position.x;
    // packet.y = msg->position.y;
    // packet.z = msg->position.z;
    // packet.yaw = msg->yaw;
    // packet.vx = msg->velocity.x;
    // packet.vy = msg->velocity.y;
    // packet.vz = msg->velocity.z;
    // packet.v_yaw = msg->v_yaw;
    // packet.r1 = msg->radius_1;
    // packet.r2 = msg->radius_2;
    // packet.dz = msg->dz;

    // 20240329 ZY: Eliminate communication latency
    // packet.cap_timestamp = time_info->time;
    packet.checksum=crc16::CRC16_Calc(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 2, UINT16_MAX);
    std::vector<uint8_t> data = toVector(packet);

    if (armor_or_buff_ == 1) {
      serial_driver_->port()->send(data);

      std_msgs::msg::Float64 latency;
      latency.data = (this->now() - armor_msg->header.stamp).seconds() * 1000.0;
      RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
      latency_pub_->publish(latency);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

// 发送Buff数据
void RMSerialDriver::sendBuffData(const buff_interfaces::msg::BuffSend::SharedPtr buff_msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    float pitch;
    float yaw;

    packet.is_fire = buff_msg->is_fire;
    packet.x = buff_msg->position.x;
    packet.y = buff_msg->position.y;
    packet.z = buff_msg->position.z;
    packet.v_yaw = 0;

    pitch = buff_msg->pitch;
    yaw = buff_msg->yaw;
    // RCLCPP_ERROR(get_logger(), "buff.yaw:%f", yaw);

    // 关于 pitch 硬补;
    packet.pitch = RMSerialDriver::angle_trans(pitch);
    packet.yaw = RMSerialDriver::angle_trans(yaw);

    // RCLCPP_ERROR(get_logger(), "packet.yaw:%f", packet.yaw);


    // SendPacket packet;
    // packet.state = msg->tracking ? 1 : 0;
    // packet.id = id_unit8_map.at(msg->id);
    // packet.armors_num = msg->armors_num;
    // packet.x = msg->position.x;
    // packet.y = msg->position.y;
    // packet.z = msg->position.z;
    // packet.yaw = msg->yaw;
    // packet.vx = msg->velocity.x;
    // packet.vy = msg->velocity.y;
    // packet.vz = msg->velocity.z;
    // packet.v_yaw = msg->v_yaw;
    // packet.r1 = msg->radius_1;
    // packet.r2 = msg->radius_2;
    // packet.dz = msg->dz;

    // 20240329 ZY: Eliminate communication latency
    // packet.cap_timestamp = time_info->time;
    packet.checksum=crc16::CRC16_Calc(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 2, UINT16_MAX);
    std::vector<uint8_t> data = toVector(packet);

    if (armor_or_buff_ == 0) {
    serial_driver_->port()->send(data);

      std_msgs::msg::Float64 latency;
      latency.data = (this->now() - buff_msg->header.stamp).seconds() * 1000.0;
      RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
      latency_pub_->publish(latency);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

// void RMSerialDriver::sendBuffData(
//   const buff_interfaces::msg::Rune::ConstSharedPtr rune,
//   const buff_interfaces::msg::TimeInfo::ConstSharedPtr time_info)
// {
//   try {
//     SendPacket packet;
//     packet.state = rune->tracking ? 2 : 0;
//     packet.id = rune->offset_id;
//     packet.armors_num = rune->offset_id;
//     packet.x = rune->position.x;
//     packet.y = rune->position.y;
//     packet.z = rune->position.z;
//     packet.yaw = rune->theta;
//     packet.vx = rune->a;
//     packet.vy = rune->b;
//     packet.vz = rune->w;
//     packet.v_yaw = 0.0;
//     packet.r1 = 0.0;
//     packet.r2 = 0.0;
//     packet.dz = 0.0;
//     packet.cap_timestamp = time_info->time;


//     if (rune->w == 0) {
//       packet.t_offset = 0;
//     } else {
//       int T = abs(2 * 3.1415926 / rune->w * 1000);
//       int offset = (rune->t_offset - time_info->time % T) % T;
//       if (offset < 0) {
//         packet.t_offset = T + offset;
//       }
//     }

//     packet.checksum=crc16::CRC16_Calc(reinterpret_cast<uint8_t *>(&packet), sizeof(packet), UINT16_MAX); 
//     // crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

//     std::vector<uint8_t> data = toVector(packet);

//     serial_driver_->port()->send(data);

//     std_msgs::msg::Float64 latency;
//     latency.data = (this->now() - rune->header.stamp).seconds() * 1000.0;
//     RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
//     latency_pub_->publish(latency);
//   } catch (const std::exception & ex) {
//     RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
//     reopenPort();
//   }
// }

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or "
        "hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

void RMSerialDriver::changeTarget()
{
  if (!change_target_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping target change");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  change_target_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Change target!");
}

//! 角度换算
// [-PI,PI] -> [0,2PI] 转换
float RMSerialDriver::angle_trans(float originAngle){

    if (originAngle < 0) {
      originAngle = originAngle +2* M_PI;
    }
    // else 
    //   originAngle = originAngle;
    return originAngle;
    // return originAngle;
}

// [0,2PI] -> [-PI,PI] 转换
float RMSerialDriver::angle_re_trans(float originAngle){

  if (originAngle <= M_PI) {
    // originAngle = originAngle;
  }
  else
    originAngle = originAngle -2*M_PI;


      return originAngle;
  // return originAngle-M_PI;
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
