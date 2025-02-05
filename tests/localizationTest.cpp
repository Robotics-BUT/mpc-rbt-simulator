#include <gtest/gtest.h>

#include <mpc-rbt-simulator/Localization.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <thread>

/// Node testing interface that will connect to the Node's API for tests
class LocalizationTester : public rclcpp::Node {
public:
    static constexpr auto PARENT_FRAME = "map";
    static constexpr auto CHILD_FRAME = "base_link";

    LocalizationTester() : Node("localization_tester") {
        joint_publisher_ =
                create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS().reliable());
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(get_logger(), "Localization test started.");
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

TEST(localizationTest, works) {
    // initialize the ROS context
    rclcpp::init(0, nullptr);

    // set up the ROS node execution
    auto node = std::make_shared<LocalizationNode>();
    auto tester = std::make_shared<LocalizationTester>();
    std::pair<std::atomic_bool, std::thread> node_thread;
    node_thread = std::make_pair(true, std::thread([&node, &tester, &node_thread] {
                                     while ((errno != EINTR) && node_thread.first.load()) {
                                         rclcpp::spin_some(tester->get_node_base_interface());
                                         rclcpp::spin_some(node->get_node_base_interface());
                                     }
                                 }));
    node_thread.second.detach();

    // perform testing logic for the test scenario
    // TODO(tests): test something reasonable
    static constexpr unsigned test_iterations = 10;
    for (unsigned i = 0; i < test_iterations; ++i) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = tester->get_clock()->now();
        msg.name.emplace_back("wheel_right_joint");
        msg.name.emplace_back("wheel_left_joint");
        msg.position.push_back(i / 10.0);
        msg.position.push_back(i / 10.0);
        msg.velocity.push_back(0.0);
        msg.velocity.push_back(0.0);
        tester->joint_publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        std::string err;
        ASSERT_TRUE(
                tester->tf_buffer_->canTransform(tester->PARENT_FRAME, tester->CHILD_FRAME, tf2::TimePointZero, &err));
        const geometry_msgs::msg::TransformStamped tf =
                tester->tf_buffer_->lookupTransform(tester->PARENT_FRAME, tester->CHILD_FRAME, tf2::TimePointZero);
        EXPECT_EQ(tf.header.frame_id, tester->PARENT_FRAME);
        EXPECT_EQ(tf.child_frame_id, tester->CHILD_FRAME);
    }

    // stop the ROS nodes and clear the context
    node_thread.first = false;
    if (node_thread.second.joinable()) node_thread.second.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
