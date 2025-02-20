#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>


namespace tiago_driver {
class TiagoDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  //TODO(message-timeout): change this if we encounter problems with the measurement (ROS time?, raw simulation time?)
  inline static rclcpp::Time getCurrentTime() { return rclcpp::Clock(RCL_STEADY_TIME).now(); }

  webots_ros2_driver::WebotsNode *node_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscriber_;

  rclcpp::Duration cmd_vel_timeout_ = rclcpp::Duration::from_seconds(1.0);
  //TODO(message-timeout): if we start using TwistStamped instead, we won't have to define this additional variable
  rclcpp::Time last_received_cmd_vel_ = getCurrentTime();
  geometry_msgs::msg::Twist cmd_vel_msg_;

  WbDeviceTag right_motor_;
  WbDeviceTag left_motor_;
  WbDeviceTag right_motor_sensor_;
  WbDeviceTag left_motor_sensor_;

  double time_step_;

  float right_motor_pos_last_;
  float left_motor_pos_last_;
};
} // namespace tiago_driver
#endif
