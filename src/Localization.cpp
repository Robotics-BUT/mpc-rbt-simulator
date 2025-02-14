#include "mpc-rbt-simulator/Localization.hpp"
#include "mpc-rbt-simulator/RobotConfig.hpp"

LocalizationNode::LocalizationNode() : rclcpp::Node("localization_node") {
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS().reliable(),
      std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {

    /********************************************************************************
     * STUDENT TASK
     * Implement odometry-based localization instead of the following placeholder.
     *******************************************************************************/
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

}
