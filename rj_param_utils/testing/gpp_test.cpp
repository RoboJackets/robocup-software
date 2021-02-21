#include <gtest/gtest.h>

#include "rj_param_utils/global_param.hpp"

namespace rj_param_utils::Testing {

class ReceiverNode : public rclcpp::Node {
public:
    ReceiverNode();

    params::ROS2GlobalParamProvider global_param_provider_ = params::ROS2GlobalParamProvider(this, "test_global_node");
};

class GlobalParamProviderTest : public ::testing::Test {
public:
    void SetUp() override {
        rclcpp::init(0, {});

        global_node_ = std::make_shared<rclcpp::Node>("test_global_node");
        receiver_node_ = std::make_shared<ReceiverNode>();
        global_node_->declare_parameter("global_bool", true);
        global_node_->declare_parameter("global_float", 3.14f);

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
    void start_ros() {
        is_running_ = true;
        ros_thread_ = std::thread{[this]() {
            while (is_running_ && rclcpp::ok()) {
                executor_->spin_once();
            }
        }};
    }

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::shared_ptr<rclcpp::Node> global_node_;
    std::shared_ptr<ReceiverNode> receiver_node_;
    std::thread ros_thread_;
    std::atomic<bool> is_running_;
};

TEST_F(GlobalParamProviderTest, request_param) {
    start_ros();
    auto receiver_bool_param = receiver_node_->global_param_provider_.params_client_->get_parameters({"global_bool"});
    auto receiver_float_param = receiver_node_->global_param_provider_.params_client_->get_parameters({"global_float"});
    ASSERT_TRUE(receiver_bool_param.get()[1].as_bool());
    ASSERT_EQ(receiver_float_param.get()[1].as_double(), 3.14f);
}
} // namespace rj_param_utils
