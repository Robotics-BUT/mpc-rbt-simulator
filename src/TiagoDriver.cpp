#include "mpc-rbt-simulator/TiagoDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.202
#define WHEEL_RADIUS 0.0985

namespace tiago_driver {
void TiagoDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  node_ = node;
  time_step_ = wb_robot_get_basic_time_step();

  right_motor_ = wb_robot_get_device("wheel_right_joint");
  left_motor_ = wb_robot_get_device("wheel_left_joint");
  wb_motor_set_position(left_motor_, INFINITY);
  wb_motor_set_velocity(left_motor_, 0.0);
  wb_motor_set_position(right_motor_, INFINITY);
  wb_motor_set_velocity(right_motor_, 0.0);

  right_motor_sensor_ = wb_robot_get_device("wheel_right_joint_sensor");
  left_motor_sensor_ = wb_robot_get_device("wheel_left_joint_sensor");
  wb_position_sensor_enable(right_motor_sensor_, 10);
  wb_position_sensor_enable(left_motor_sensor_, 10);

  cmd_vel_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&TiagoDriver::cmdVelCallback, this, std::placeholders::_1));
  
  joint_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  right_motor_pos_last_ = wb_position_sensor_get_value(right_motor_sensor_);
  left_motor_pos_last_ = wb_position_sensor_get_value(left_motor_sensor_);
}

void TiagoDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg_.linear = msg->linear;
  cmd_vel_msg_.angular = msg->angular;
}

// the step() method is called at every time step of the simulation
void TiagoDriver::step() {

  // inverse Kinematics - compute the motor speeds based on the cmd_vel
  auto forward_speed = cmd_vel_msg_.linear.x;
  auto angular_speed = cmd_vel_msg_.angular.z;
  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  // update motors
  wb_motor_set_velocity(left_motor_, command_motor_left);
  wb_motor_set_velocity(right_motor_, command_motor_right);

  // read motor position
  auto right_motor_pos = wb_position_sensor_get_value(right_motor_sensor_);
  auto left_motor_pos = wb_position_sensor_get_value(left_motor_sensor_);

  // compute motor angular velocity
  auto right_motor_vel = (right_motor_pos - right_motor_pos_last_) / (time_step_ * 0.001);
  auto left_motor_vel = (left_motor_pos - left_motor_pos_last_) / (time_step_ * 0.001);
  right_motor_pos_last_ = right_motor_pos;
  left_motor_pos_last_ = left_motor_pos;

  // publish motor joint states
  auto message = sensor_msgs::msg::JointState();
  message.header.stamp = node_->get_clock()->now();
  message.name.push_back("wheel_right_joint");
  message.name.push_back("wheel_left_joint");
  message.position.push_back(right_motor_pos);
  message.position.push_back(left_motor_pos);
  message.velocity.push_back(right_motor_vel);
  message.velocity.push_back(left_motor_vel);
  joint_publisher_->publish(message);
}
} // namespace tiago_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tiago_driver::TiagoDriver,
                       webots_ros2_driver::PluginInterface)