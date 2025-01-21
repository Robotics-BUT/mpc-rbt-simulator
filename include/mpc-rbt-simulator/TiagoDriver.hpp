#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"


namespace tiago_driver {
class TiagoDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  webots_ros2_driver::WebotsNode *node_;
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;

  geometry_msgs::msg::Twist cmd_vel_msg;

  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
  WbDeviceTag right_motor_sensor;
  WbDeviceTag left_motor_sensor;
   
};
} // namespace tiago_driver
#endif