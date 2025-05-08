// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <auto_aim_interfaces/msg/detail/receive__struct.hpp>
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_solve/solve.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_interfaces/msg/receive.hpp"

#include "auto_aim_interfaces/msg/all_latency.hpp"

#include "std_msgs/msg/float64.hpp"
#include "auto_aim_interfaces/msg/test.hpp"

namespace rm_auto_aim
{
class ArmorSolveNode : public rclcpp::Node
{
public:
    explicit ArmorSolveNode(const rclcpp::NodeOptions & options);
private:
    void velocityCallback(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg);
    void receiveCallback(const auto_aim_interfaces::msg::Receive::SharedPtr receive_msg);
    void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg);
    void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);   //

    // solve
    std::unique_ptr<Solve> solve_;


    // solve parameter
    float k;
    int bias_time;
    float s_bias;
    float z_bias;

    float v_top;

    // init velocity
    float current_v;

    // Publisher
    rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Subscriber 
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Receive>::SharedPtr receive_sub_;


};
}   // namespace rm_auto_aim