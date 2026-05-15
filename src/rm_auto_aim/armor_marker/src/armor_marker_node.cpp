#include "armor_marker/armor_marker_node.hpp"

#include <cmath>
#include <string>

namespace rm_auto_aim
{
ArmorMarkerNode::ArmorMarkerNode(const rclcpp::NodeOptions& options)
    : Node("armor_marker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting ArmorMarkerNode!");

  // ===================== TrajectorySolver 初始化 =====================
  // marker 节点仅使用 PredictCenter / PredictArmor 做整车建模，
  // 不需要实际弹道解算，因此各解算参数置零，模式设为 NORMAL 跳过查表。
  Table::TableConfig dummy_config{0, 0, 0, 0, 0, ""};
  solver_ = std::make_unique<TrajectorySolver>(
      /*k=*/0.0, /*bias_time=*/0.0, /*s_bias=*/0.0, /*z_bias=*/0.0,
      /*pitch_bias=*/0.0, TrajectorySolver::CalculateMode::NORMAL,
      dummy_config, dummy_config);

  // ===================== Detector Marker 初始化 =====================
  // 从 detector_node 中分离：可视化检测到的装甲板
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  detector_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detector/marker", 10);

  // 默认保持原逻辑：显示 /detector/armors（只含可见板）。
  // 若要显示仿真中的所有板，可将该参数设为 /ground_truth/armors。
  // const std::string detector_armors_topic =
  //     this->declare_parameter("detector_marker.armors_topic",
  //                             std::string("/ground_truth/armors"));
 const std::string detector_armors_topic =
      this->declare_parameter("detector_marker.armors_topic",
                              std::string("/detector/armors"));

  armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
      detector_armors_topic, rclcpp::SensorDataQoS(),
      std::bind(&ArmorMarkerNode::DetectorArmorsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Detector marker subscribes to: %s",
              detector_armors_topic.c_str());

  // ===================== Tracker Marker 初始化 =====================
  // 从 tracker_node 中分离：可视化跟踪状态
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;

  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;

  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;

  tracker_armor_marker_.ns = "tracked_armors";
  tracker_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  tracker_armor_marker_.scale.x = 0.03;
  tracker_armor_marker_.scale.z = 0.125;
  tracker_armor_marker_.color.a = 1.0;
  tracker_armor_marker_.color.r = 1.0;

  // 瞄准点 marker，由 trajectory info 驱动
  aiming_point_marker_.ns = "aiming_point";
  aiming_point_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_marker_.scale.x = aiming_point_marker_.scale.y =
      aiming_point_marker_.scale.z = 0.12;
  aiming_point_marker_.color.a = 1.0;
  aiming_point_marker_.color.r = 1.0;
  aiming_point_marker_.color.b = 1.0;
  aiming_point_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  tracker_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tracker/marker", 10);

  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", rclcpp::SensorDataQoS(),
      std::bind(&ArmorMarkerNode::TrackerTargetCallback, this, std::placeholders::_1));

  // ===================== Trajectory Info 订阅 =====================
  trajectory_info_sub_ =
      this->create_subscription<auto_aim_interfaces::msg::TrajectoryInfo>(
          "/trajectory/info", rclcpp::SensorDataQoS(),
          std::bind(&ArmorMarkerNode::TrajectoryInfoCallback, this,
                    std::placeholders::_1));
}

// ======================== Detector Marker ========================
void ArmorMarkerNode::DetectorArmorsCallback(
    const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  armor_marker_.header = armors_msg->header;
  text_marker_.header = armors_msg->header;

  armor_marker_.id = 0;
  text_marker_.id = 0;

  if (armors_msg->armors.empty())
  {
    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(armor_marker_);
  }
  else
  {
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.action = visualization_msgs::msg::Marker::ADD;

    for (const auto& armor : armors_msg->armors)
    {
      armor_marker_.id++;
      armor_marker_.scale.y = (armor.type == "small") ? 0.135 : 0.23;
      armor_marker_.pose = armor.pose;

      text_marker_.id++;
      text_marker_.pose.position = armor.pose.position;
      text_marker_.pose.position.y -= 0.1;
      text_marker_.text = armor.number;

      marker_array.markers.emplace_back(armor_marker_);
      marker_array.markers.emplace_back(text_marker_);
    }
  }

  detector_marker_pub_->publish(marker_array);
}

// ======================== Tracker Marker ========================
void ArmorMarkerNode::TrackerTargetCallback(
    const auto_aim_interfaces::msg::Target::SharedPtr target_msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  position_marker_.header = target_msg->header;
  linear_v_marker_.header = target_msg->header;
  angular_v_marker_.header = target_msg->header;
  tracker_armor_marker_.header = target_msg->header;

  if (target_msg->tracking)
  {
    double xc = target_msg->position.x;
    double yc = target_msg->position.y;
    double za = target_msg->position.z;
    double vx = target_msg->velocity.x;
    double vy = target_msg->velocity.y;
    double vz = target_msg->velocity.z;
    double dz = target_msg->dz;

    // ---------- 旋转中心球体 ----------
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    // ---------- 线速度箭头 ----------
    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    // ---------- 角速度箭头 ----------
    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg->v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    // ---------- 使用 TrajectorySolver 整车建模预测各装甲板位置 ----------
    // 构造 solver 内部 Target，同 tracker / trajectory_node 的用法
    TrajectorySolver::Target target;
    target.position.x = target_msg->position.x;
    target.position.y = target_msg->position.y;
    target.position.z = target_msg->position.z;
    target.position.yaw = target_msg->yaw;
    target.velocity.x = target_msg->velocity.x;
    target.velocity.y = target_msg->velocity.y;
    target.velocity.z = target_msg->velocity.z;
    target.velocity.yaw = target_msg->v_yaw;
    target.num = target_msg->armors_num;
    target.type = target_msg->type;
    target.radius1 = target_msg->radius_1;
    target.radius2 = target_msg->radius_2;
    target.outpost_idx = target_msg->outpost_idx;
    solver_->SetTarget(target);

    // time_delay = 0：可视化当前时刻的装甲板位置
    TrajectorySolver::TarPostion pre_center = solver_->PredictCenter(0);

    int a_n = target_msg->armors_num;
    tracker_armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    tracker_armor_marker_.scale.y = (target_msg->type == "small") ? 0.135 : 0.23;

    for (int i = 0; i < a_n; i++)
    {
      TrajectorySolver::TarPostion armor_pos = solver_->PredictArmor(i, pre_center);

      geometry_msgs::msg::Point p_a;
      p_a.x = armor_pos.x;
      p_a.y = armor_pos.y;
      p_a.z = armor_pos.z;

      tracker_armor_marker_.id = i;
      tracker_armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      // 前哨站装甲板向下倾斜，其余向上
      q.setRPY(0, target_msg->type == "outpost" ? -0.26 : 0.26, armor_pos.yaw);
      tracker_armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(tracker_armor_marker_);
    }
  }
  else
  {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    tracker_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(tracker_armor_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  tracker_marker_pub_->publish(marker_array);
}

// ======================== Trajectory Info Marker ========================
void ArmorMarkerNode::TrajectoryInfoCallback(
    const auto_aim_interfaces::msg::TrajectoryInfo::SharedPtr info_msg)
{
  // 仅当瞄准点有效时发布
  if (std::fabs(info_msg->aim_position.x) < 0.01)
  {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  aiming_point_marker_.header.stamp = this->now();
  aiming_point_marker_.header.frame_id = "gimbal_odom";
  aiming_point_marker_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_marker_.pose.position = info_msg->aim_position;
  marker_array.markers.emplace_back(aiming_point_marker_);
  tracker_marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its
// library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorMarkerNode)
