#include "rj_param_utils/global_param_provider.hpp"

#include <gtest/gtest.h>

#include "rj_param_utils/param.hpp"

namespace params::testing {

DEFINE_BOOL(kGlobalModule, global_bool, true, "Some sample boolean");

class GlobalParamProviderTest : public ::testing::Test {
public:
    void SetUp() override {
        rclcpp::init(0, {});

        global_node_ = std::make_shared<rclcpp::Node>(
            "test_global_node", rclcpp::NodeOptions{}
                                    .allow_undeclared_parameters(true)
                                    .automatically_declare_parameters_from_overrides(true));
        receiver_node_ = std::make_shared<rclcpp::Node>("test_receiver_node");
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(global_node_);
        executor_->add_node(receiver_node_);
    }

    void TearDown() override {
        is_running_ = false;
        ros_thread_.join();
        rclcpp::shutdown();
    }

protected:
    void init_param_provider() {
        executor_->remove_node(receiver_node_);
        param_provider_ = std::make_shared<ROS2GlobalParamProvider>(
            receiver_node_.get(), kGlobalModule, "test_global_node");
        executor_->add_node(receiver_node_);
    }

    void start_ros() {
        is_running_ = true;
        ros_thread_ = std::thread{[this]() {
            while (is_running_ && rclcpp::ok()) {
                executor_->spin_some(std::chrono::milliseconds(100));
            }
        }};
    }

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::shared_ptr<rclcpp::Node> global_node_;
    std::shared_ptr<rclcpp::Node> receiver_node_;
    std::shared_ptr<ROS2GlobalParamProvider> param_provider_;
    std::thread ros_thread_;
    std::atomic<bool> is_running_;
};

/**
 * @brief Test that the default value of DEFINE_* variant of defining
 * params is correct.
 */
TEST_F(GlobalParamProviderTest, default_parameter_values) {
    init_param_provider();
    start_ros();
    ASSERT_TRUE(PARAM_global_bool);
}

/**
 * @brief Test that the default value of the DEFINE_NS_* variant of
 * defining params is correct.
 */
TEST_F(GlobalParamProviderTest, parameter_updates) {
    init_param_provider();
    start_ros();
    EXPECT_TRUE(PARAM_global_bool);
    ASSERT_TRUE(global_node_->set_parameter(rclcpp::Parameter("global_bool", false)).successful);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_FALSE(PARAM_global_bool);
}

/**
 * @brief Test that the default value of the DEFINE_NS_* variant of
 * defining params is correct.
 */
TEST_F(GlobalParamProviderTest, initial_value_on_global_server) {
    start_ros();
    ASSERT_TRUE(global_node_->set_parameter(rclcpp::Parameter("global_bool", false)).successful);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    init_param_provider();
    EXPECT_FALSE(PARAM_global_bool);
}

}  // namespace params::testing
