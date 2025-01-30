#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode();

private:
    void jointCallback(const sensor_msgs::msg::JointState & msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // LOCALIZATION_HPP