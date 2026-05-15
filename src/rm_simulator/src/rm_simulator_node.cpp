// ============================================================================
//  armor_simulator_node.cpp
//
//  ROS2 仿真组件节点 —— 配合 tracker_node 使用
//
//  发布:
//    /detector/armors           带噪声的装甲板 (SensorDataQoS, 供 tracker 订阅)
//    /ground_truth/armors       无噪声真值装甲板 (可选全部装甲板, 供对比/可视化)
//    /ground_truth/noisy_armors 带噪声真值装甲板 (可选全部装甲板, 供对比/可视化)
//    /ground_truth/robot_pose   车体中心真值 (camera_optical_frame 下)
//
//  可选静态 TF (publish_tf=true 时发布, 匹配 rm_gimbal.urdf.xacro, 关节角归零):
//    odom → gimbal_odom → yaw_link → pitch_link
//         → camera_link → camera_optical_frame
//
// ============================================================================

#include <tf2_ros/static_transform_broadcaster.h>

#include <auto_aim_interfaces/msg/armor.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "rm_simulator/robomaster.h"

using namespace std::chrono_literals;

namespace rm_auto_aim
{

// ============================================================================
//  ArmorSimulatorNode
// ============================================================================

class RmSimulatorNode : public rclcpp::Node
{
 public:
  explicit RmSimulatorNode(const rclcpp::NodeOptions& options)
      : Node("armor_simulator", options)
  {
    // ---------- 参数声明 ----------

    // 发布频率 (Hz)
    double rate = declare_parameter("publish_rate", 100.0);

    // 是否发布静态 TF (纯仿真=true, 真云台=false 由 robot_state_publisher 负责)
    bool publish_tf = declare_parameter("publish_tf", true);

    // ground_truth 话题是否发布所有装甲板（包括不可见板）。
    // /detector/armors 仍只发布可见板，避免影响 tracker。
    gt_publish_all_armors_ = declare_parameter("ground_truth.publish_all_armors", true);

    // 机器人初始位姿 (相机光学坐标系: X 右, Y 下, Z 前)
    double init_x = declare_parameter("robot.init_x", 0.0);
    double init_y = declare_parameter("robot.init_y", 0.0);
    double init_z = declare_parameter("robot.init_z", 3.0);  // 5 m 正前方
    double init_yaw = declare_parameter("robot.init_yaw", M_PI / 2.0);

    // 车体坐标系速度 (常量, 可通过参数调节)
    double vx = declare_parameter("robot.vx", 0.0);        // 横移 (右)
    double vy = declare_parameter("robot.vy", 0.0);        // 升降 (下)
    double vz = declare_parameter("robot.vz", 0.0);        // 前进
    double omega = declare_parameter("robot.omega", 0.0);  // 偏航角速度 (rad/s)

    // 装甲板配置
    int armor_count = declare_parameter("robot.armor_count", 4);
    double horizontal_dist = declare_parameter("robot.horizontal_dist", 0.20);
    double height_offset = declare_parameter("robot.height_offset", 0.0);
    double armor_pitch = declare_parameter("robot.armor_pitch", M_PI / 12.0);

    // 线性噪声: sigma = k0 + k1 * distance
    double k0_pos = declare_parameter("noise.k0_pos", 0.000);
    double k1_pos = declare_parameter("noise.k1_pos", 0.01);
    double k0_ori = declare_parameter("noise.k0_ori", 0.0);
    double k1_ori = declare_parameter("noise.k1_ori", 0.03);
    int noise_seed = declare_parameter("noise.seed", -1);  // <0 则不固定

    // 相机外参 (对应 URDF 中 camera_joint 的 xyz)
    cam_x_ = declare_parameter("camera.x", 0.10);
    cam_y_ = declare_parameter("camera.y", 0.0);
    cam_z_ = declare_parameter("camera.z", 0.05);

    // ---------- 初始化仿真器 ----------

    robot_.setInitialPose(init_x, init_y, init_z, init_yaw);

    if (armor_count == 3)
      robot_.setupTriArmors(horizontal_dist, height_offset, armor_pitch);
    else
      robot_.setupDefaultArmors(horizontal_dist, height_offset, armor_pitch);

    robot_.setVelocityX([vx](double t) { return M_PI / 2.0 * cos(M_PI / 2.0 * t); });
    //robot_.setVelocityX([vx](double t) { return vx; });
    robot_.setVelocityY([vy](double) { return vy; });
    robot_.setVelocityZ([vz](double) { return vz; });
    robot_.setAngularVelocity([omega](double) { return omega; });

    noise_ = NoiseConfig::LinearNoise(k0_pos, k1_pos, k0_ori, k1_ori);

    if (noise_seed >= 0) robot_.setNoiseSeed(static_cast<unsigned int>(noise_seed));

    // ---------- 发布者 ----------

    // 供 tracker_node 订阅 (SensorDataQoS 与 tracker 的订阅一致)
    armors_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS());

    // 真值对比用
    gt_armors_pub_ =
        create_publisher<auto_aim_interfaces::msg::Armors>("/ground_truth/armors", 10);
    noisy_armors_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
        "/ground_truth/noisy_armors", 10);
    gt_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth/robot_pose", 10);

    // ---------- 可选静态 TF ----------

    if (publish_tf)
    {
      publishStaticTFs();
      RCLCPP_INFO(get_logger(), "Static TFs published (pure simulation mode)");
    }
    else
    {
      RCLCPP_INFO(
          get_logger(),
          "TF publishing disabled (expecting robot_state_publisher + serial_driver)");
    }

    // ---------- 定时器 ----------

    dt_ = 1.0 / rate;
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                               std::bind(&RmSimulatorNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
                "Armor simulator started: rate=%.0f Hz, robot=(%.2f,%.2f,%.2f), "
                "omega=%.2f rad/s, armor_count=%d, publish_tf=%s, "
                "ground_truth.publish_all_armors=%s",
                rate, init_x, init_y, init_z, omega, armor_count,
                publish_tf ? "true" : "false",
                gt_publish_all_armors_ ? "true" : "false");
  }

 private:
  // ==========================================================================
  //  定时回调: 步进仿真 → 发布所有话题
  // ==========================================================================
  void timerCallback()
  {
    robot_.update(dt_);

    auto stamp = now();

    // detector 输入仍然只发布可见板，避免不可见板进入 tracker。
    Armors detector_noisy = robot_.getArmorsWithNoise(noise_);
    armors_pub_->publish(toArmorsMsg(detector_noisy, stamp));

    // ground_truth 话题可选发布所有板，供 RViz/Marker 可视化使用。
    const bool gt_visible_only = !gt_publish_all_armors_;
    Armors gt = robot_.getArmors(nullptr, false);
    Armors noisy_gt = robot_.getArmorsWithNoise(noise_, false);

    // 真值对比/可视化
    gt_armors_pub_->publish(toArmorsMsg(gt, stamp));
    noisy_armors_pub_->publish(toArmorsMsg(noisy_gt, stamp));

    // 车体中心真值 (camera_optical_frame 下)
    publishRobotPose(stamp);
  }

  // ==========================================================================
  //  Armors → ROS 消息
  // ==========================================================================
  auto_aim_interfaces::msg::Armors toArmorsMsg(const Armors& armors,
                                               const rclcpp::Time& stamp) const
  {
    auto_aim_interfaces::msg::Armors msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera_optical_frame";

    for (const auto& a : armors.armors)
    {
      auto_aim_interfaces::msg::Armor armor_msg;
      armor_msg.number = a.number;
      armor_msg.type = a.type;
      armor_msg.distance_to_image_center = a.distance_to_image_center;

      armor_msg.pose.position.x = a.pose.position.x;
      armor_msg.pose.position.y = a.pose.position.y;
      armor_msg.pose.position.z = a.pose.position.z;

      armor_msg.pose.orientation.x = a.pose.orientation.x;
      armor_msg.pose.orientation.y = a.pose.orientation.y;
      armor_msg.pose.orientation.z = a.pose.orientation.z;
      armor_msg.pose.orientation.w = a.pose.orientation.w;

      msg.armors.push_back(std::move(armor_msg));
    }
    return msg;
  }

  // ==========================================================================
  //  发布车体中心真值
  // ==========================================================================
  void publishRobotPose(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera_optical_frame";

    msg.pose.position.x = -robot_.x();
    msg.pose.position.y = robot_.y();
    msg.pose.position.z = robot_.z();

    // yaw 绕 Y 轴 → 四元数
    double half = robot_.yaw() / 2.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = std::sin(half);
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = std::cos(half);

    gt_pose_pub_->publish(msg);
  }

  // ==========================================================================
  //  发布静态 TF (匹配 rm_gimbal.urdf.xacro, 所有关节角 = 0)
  //
  //  链路:
  //    odom ──(identity)──▷ gimbal_odom
  //         ──(identity)──▷ yaw_link      (yaw_joint = 0)
  //         ──(identity)──▷ pitch_link    (pitch_joint = 0)
  //         ──(xyz)──────▷ camera_link    (camera_joint, 平移)
  //         ──(rpy)──────▷ camera_optical_frame  (rpy = -π/2, 0, -π/2)
  //
  // ==========================================================================
  void publishStaticTFs()
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    auto stamp = now();

    auto make_identity = [&](const std::string& parent, const std::string& child)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = parent;
      t.child_frame_id = child;
      t.transform.rotation.w = 1.0;
      return t;
    };

    // odom → gimbal_odom
    tfs.push_back(make_identity("odom", "gimbal_odom"));

    // gimbal_odom → yaw_link (yaw = 0)
    tfs.push_back(make_identity("gimbal_odom", "yaw_link"));

    // yaw_link → pitch_link (pitch = 0)
    tfs.push_back(make_identity("yaw_link", "pitch_link"));

    // pitch_link → camera_link (平移, 对应 URDF 中 camera_joint 的 xyz)
    {
      auto t = make_identity("pitch_link", "camera_link");
      t.transform.translation.x = cam_x_;
      t.transform.translation.y = cam_y_;
      t.transform.translation.z = cam_z_;
      tfs.push_back(t);
    }

    // camera_link → camera_optical_frame
    //   rpy = (-π/2, 0, -π/2)
    //   对应四元数 q = (-0.5, 0.5, -0.5, 0.5)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = "camera_link";
      t.child_frame_id = "camera_optical_frame";
      t.transform.rotation.x = -0.5;
      t.transform.rotation.y = 0.5;
      t.transform.rotation.z = -0.5;
      t.transform.rotation.w = 0.5;
      tfs.push_back(t);
    }

    tf_broadcaster_->sendTransform(tfs);
    RCLCPP_INFO(get_logger(),
                "Published %zu static transforms (odom -> camera_optical_frame)",
                tfs.size());
  }

  // ==========================================================================
  //  成员
  // ==========================================================================
  RoboMaster robot_;
  NoiseConfig noise_;
  double dt_ = 0.01;
  double cam_x_ = 0.10, cam_y_ = 0.0, cam_z_ = 0.05;
  bool gt_publish_all_armors_ = true;

  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr gt_armors_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr noisy_armors_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_pub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RmSimulatorNode)
