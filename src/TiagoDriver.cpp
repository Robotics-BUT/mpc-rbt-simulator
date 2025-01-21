#include "mpc-rbt-simulator/TiagoDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025

namespace tiago_driver {
void TiagoDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  node_ = node;

  right_motor = wb_robot_get_device("wheel_right_joint");
  left_motor = wb_robot_get_device("wheel_left_joint");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  right_motor_sensor = wb_robot_get_device("wheel_right_joint_sensor");
  left_motor_sensor = wb_robot_get_device("wheel_left_joint_sensor");
  wb_position_sensor_enable(right_motor_sensor, 10);
  wb_position_sensor_enable(left_motor_sensor, 10);

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&TiagoDriver::cmdVelCallback, this, std::placeholders::_1));
  
  joint_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
}

void TiagoDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

void TiagoDriver::step() {
  // kinematics
  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;
  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  // update motors
  wb_motor_set_velocity(left_motor, command_motor_left);
  wb_motor_set_velocity(right_motor, command_motor_right);

  // read motor position
  auto right_motor_position = wb_position_sensor_get_value(right_motor_sensor);
  auto left_motor_position = wb_position_sensor_get_value(left_motor_sensor);

  // TODO compute motor angular velocity

  // publish motor joint states
  auto message = sensor_msgs::msg::JointState();
  message.header.stamp = node_->get_clock()->now();
  message.name.push_back("wheel_right_joint");
  message.name.push_back("wheel_left_joint");
  message.position.push_back(right_motor_position);
  message.position.push_back(left_motor_position);
  joint_publisher_->publish(message);
}
} // namespace tiago_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tiago_driver::TiagoDriver,
                       webots_ros2_driver::PluginInterface)