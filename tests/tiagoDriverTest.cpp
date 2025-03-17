#include <utility>

#include <gtest/gtest.h>

#include <webots/robot.h>

#include <mpc_rbt_simulator/TiagoDriver.hpp>

//NOTE: WebotsNode is not compiled as a library, but only as an executable, so we don't actually have access to the definition
// and must create our own one.
namespace webots_ros2_driver {
    WebotsNode::WebotsNode(std::string name)
        : Node(name), mSetRobotStatePublisher(false), mStep(0),
          mPluginLoader("webots_ros2_driver", "webots_ros2_driver::PluginInterface"), mWebotsXMLElement(nullptr) {}
}// namespace webots_ros2_driver

/// Node testing interface that will connect to the Node's API for tests
class TiagoDriverTester : public rclcpp::Node {
public:
    static constexpr auto ROBOT_NAME = "tiago_base";
    const std::pair<const char *const, std::string> WEBOTS_CONTROLLER_URL{
            "WEBOTS_CONTROLLER_URL",
            std::string("ipc://") + (!getenv("WEBOTS_CONTROLLER_PORT") ? "1234" : getenv("WEBOTS_CONTROLLER_PORT")) +
                    '/' + ROBOT_NAME};
    static constexpr uint64_t DEFAULT_CMD_VEL_TIMEOUT_MILLIS = 1000;

    explicit TiagoDriverTester(std::function<void(sensor_msgs::msg::JointState::SharedPtr)> state_callback)
        : Node("tiago_driver_tester"), state_callback_(std::move(state_callback)) {
        joint_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", rclcpp::SensorDataQoS().reliable(),
                std::bind(&TiagoDriverTester::stateCallback, this, std::placeholders::_1));
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS().reliable());

        RCLCPP_INFO(get_logger(), "Tiago Driver test started.");
    }

    inline void initWebotsControllerMock() {
        setenv(WEBOTS_CONTROLLER_URL.first, WEBOTS_CONTROLLER_URL.second.c_str(), 0);
        wb_robot_init();
    }

    static inline void stepWebotsControllerMock() { wb_robot_step(static_cast<int>(wb_robot_get_basic_time_step())); }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    void stateCallback(sensor_msgs::msg::JointState::SharedPtr msg) const { state_callback_(std::move(msg)); }
    std::function<void(sensor_msgs::msg::JointState::SharedPtr)> state_callback_;
};

TEST(tiagoDriverTest, works) {
    // set up the ROS node execution
    std::pair<sensor_msgs::msg::JointState, std::mutex> last_state;
    auto node = std::make_shared<webots_ros2_driver::WebotsNode>("webots_test");
    auto tester = std::make_shared<TiagoDriverTester>([&last_state](sensor_msgs::msg::JointState::SharedPtr msg) {
        // The robot node should ALWAYS publish a state with valid data fields.
        EXPECT_EQ(msg->name.size(), 2u);
        EXPECT_EQ(msg->name[0], "wheel_right_joint");
        EXPECT_EQ(msg->name[1], "wheel_left_joint");
        EXPECT_EQ(msg->position.size(), 2u);
        EXPECT_EQ(msg->velocity.size(), 2u);
        const std::lock_guard<std::mutex> lock(last_state.second);
        last_state.first = *msg;
    });
    auto driver = tiago_driver::TiagoDriver();
    std::unordered_map<std::string, std::string> params{};
    tester->initWebotsControllerMock();
    driver.init(node.get(), params);
    std::pair<std::atomic_bool, std::thread> node_thread;
    node_thread = std::make_pair(true, std::thread([&node, &driver, &tester, &node_thread] {
                                     while ((errno != EINTR) && node_thread.first.load()) {
                                         tester->stepWebotsControllerMock();
                                         driver.step();
                                         rclcpp::spin_some(tester->get_node_base_interface());
                                         rclcpp::spin_some(node->get_node_base_interface());
                                     }
                                 }));
    node_thread.second.detach();

    // perform logic for the test scenario
    static constexpr unsigned publishing_iterations = 40;
    // IF the tester publishes positive velocity command messages ...
    geometry_msgs::msg::Twist msg;
    for (unsigned i = 0; i < publishing_iterations; ++i) {
        msg.linear.x = 0.5;
        tester->cmd_vel_publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // ... THEN the robot node should end up with a positive non-zero velocity state ...
    {
        const std::lock_guard<std::mutex> lock(last_state.second);
        EXPECT_GT(static_cast<int>(last_state.first.velocity[0]), 0);
        EXPECT_GT(static_cast<int>(last_state.first.velocity[1]), 0);
    }
    // ... THEN IF the tester publishes zero velocity command messages ...
    for (unsigned i = 0; i < publishing_iterations; ++i) {
        msg.linear.x = 0.0;
        tester->cmd_vel_publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // ... THEN the robot node should end up with a zero velocity state ...
    {
        const std::lock_guard<std::mutex> lock(last_state.second);
        EXPECT_EQ(static_cast<int>(last_state.first.velocity[0]), 0);
        EXPECT_EQ(static_cast<int>(last_state.first.velocity[1]), 0);
    }
    // ... THEN IF the tester publishes negative velocity command messages ...
    for (unsigned i = 0; i < publishing_iterations; ++i) {
        msg.linear.x = -0.5;
        tester->cmd_vel_publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // ... THEN the robot node should end up with a negative non-zero velocity state ...
    {
        const std::lock_guard<std::mutex> lock(last_state.second);
        EXPECT_LT(static_cast<int>(last_state.first.velocity[0]), 0);
        EXPECT_LT(static_cast<int>(last_state.first.velocity[1]), 0);
    }
    // ... THEN IF tester publishes zero velocity command messages ...
    for (unsigned i = 0; i < publishing_iterations; ++i) {
        msg.linear.x = 0.0;
        tester->cmd_vel_publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // ... THEN the robot node should end up with a zero velocity state.
    {
        const std::lock_guard<std::mutex> lock(last_state.second);
        EXPECT_EQ(static_cast<int>(last_state.first.velocity[0]), 0);
        EXPECT_EQ(static_cast<int>(last_state.first.velocity[1]), 0);
    }

    // stop the ROS nodes
    node_thread.first = false;
    if (node_thread.second.joinable()) node_thread.second.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST(tiagoDriverTest, cmdVelTimeout) {
    // set up the ROS node execution
    std::pair<sensor_msgs::msg::JointState, std::mutex> last_state;
    auto node = std::make_shared<webots_ros2_driver::WebotsNode>("webots_test");
    auto tester = std::make_shared<TiagoDriverTester>([&last_state](sensor_msgs::msg::JointState::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(last_state.second);
        last_state.first = *msg;
    });
    auto driver = tiago_driver::TiagoDriver();
    std::unordered_map<std::string, std::string> params{};
    tester->initWebotsControllerMock();
    driver.init(node.get(), params);
    std::pair<std::atomic_bool, std::thread> node_thread;
    node_thread = std::make_pair(true, std::thread([&node, &driver, &tester, &node_thread] {
                                     while ((errno != EINTR) && node_thread.first.load()) {
                                         tester->stepWebotsControllerMock();
                                         driver.step();
                                         rclcpp::spin_some(tester->get_node_base_interface());
                                         rclcpp::spin_some(node->get_node_base_interface());
                                     }
                                 }));
    node_thread.second.detach();

    // perform logic for the test scenario
    static constexpr unsigned test_iterations = 3;
    static constexpr unsigned publishing_iterations = 40;
    geometry_msgs::msg::Twist msg;
    msg.linear.x = -0.5;
    for (unsigned i = 0; i < test_iterations; ++i) {
        // IF the tester publishes some non-zero velocity command messages ...
        msg.linear.x = -msg.linear.x;
        for (unsigned _ = 0; _ < publishing_iterations; ++_) {
            tester->cmd_vel_publisher_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        // ... THEN the robot node should end up with a non-zero velocity state ...
        {
            const std::lock_guard<std::mutex> lock(last_state.second);
            ASSERT_NE(static_cast<int>(last_state.first.velocity[0]), 0);
            ASSERT_NE(static_cast<int>(last_state.first.velocity[1]), 0);
        }
        // ... THEN IF no velocity command messages are published for longer that the timeout duration ...
        std::this_thread::sleep_for(std::chrono::milliseconds(3 * tester->DEFAULT_CMD_VEL_TIMEOUT_MILLIS));
        // ... THEN the robot node should end up with a zero velocity state.
        {
            const std::lock_guard<std::mutex> lock(last_state.second);
            ASSERT_EQ(static_cast<int>(last_state.first.velocity[0]), 0);
            ASSERT_EQ(static_cast<int>(last_state.first.velocity[1]), 0);
        }
    }

    // stop the ROS nodes
    node_thread.first = false;
    if (node_thread.second.joinable()) node_thread.second.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

//TODO: add more tests

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    const auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
