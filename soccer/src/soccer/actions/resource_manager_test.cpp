#include "resource_manager.hpp"

#include <gtest/gtest.h>

namespace actions {

class ResourceManagerIntegrationTest : public ::testing::Test {
public:
    void SetUp() override {
        rclcpp::init(0, {});

        resource_manager_ = std::make_shared<ResourceManager>();
        client_node_ = std::make_shared<rclcpp::Node>("test_resource_client_node");

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        executor_->add_node(resource_manager_);
        executor_->add_node(client_node_);

        client_ = rclcpp_action::create_client<rj_msgs::action::UseResource>(client_node_,
                                                                             "resource_manager");
    }

    void TearDown() override {
        // Cancel goals. This is necessary because of dumb race conditions in the actionlib - the
        // goal handles need to be destroyed before the object is killed.
        client_->async_cancel_all_goals().wait();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

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
    std::shared_ptr<ResourceManager> resource_manager_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp_action::Client<rj_msgs::action::UseResource>::SharedPtr client_;
    std::thread ros_thread_;
    std::atomic<bool> is_running_;
};

TEST_F(ResourceManagerIntegrationTest, request_resource) {
    start_ros();
    auto future = client_->async_send_goal(
        rj_msgs::build<rj_msgs::action::UseResource::Goal>().priority(0).resources_requested({
            rj_msgs::build<rj_msgs::msg::Resource>().robot_id(0).subsystem(0),
        }));
    future.wait();
    ASSERT_TRUE(future.valid());
    ASSERT_EQ(future.get()->get_status(), rclcpp_action::GoalStatus::STATUS_ACCEPTED);
}

TEST_F(ResourceManagerIntegrationTest, conflict_with_identical_requests) {
    start_ros();
    auto future_1 = client_->async_send_goal(
        rj_msgs::build<rj_msgs::action::UseResource::Goal>().priority(1).resources_requested({
            rj_msgs::build<rj_msgs::msg::Resource>().robot_id(0).subsystem(0),
        }));
    future_1.wait();
    ASSERT_TRUE(future_1.valid());
    ASSERT_EQ(future_1.get()->get_status(), rclcpp_action::GoalStatus::STATUS_ACCEPTED);
    auto future_2 = client_->async_send_goal(
        rj_msgs::build<rj_msgs::action::UseResource::Goal>().priority(0).resources_requested({
            rj_msgs::build<rj_msgs::msg::Resource>().robot_id(0).subsystem(0),
        }));
    future_2.wait();
    ASSERT_TRUE(future_2.valid());
    ASSERT_EQ(future_2.get(), nullptr);
}

TEST_F(ResourceManagerIntegrationTest, preempt_with_higher_priority) {
    start_ros();
    auto future_1 = client_->async_send_goal(
        rj_msgs::build<rj_msgs::action::UseResource::Goal>().priority(0).resources_requested({
            rj_msgs::build<rj_msgs::msg::Resource>().robot_id(0).subsystem(0),
        }));
    future_1.wait();
    ASSERT_TRUE(future_1.valid());
    ASSERT_EQ(future_1.get()->get_status(), rclcpp_action::GoalStatus::STATUS_ACCEPTED);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto future_2 = client_->async_send_goal(
        rj_msgs::build<rj_msgs::action::UseResource::Goal>().priority(0).resources_requested({
            rj_msgs::build<rj_msgs::msg::Resource>().robot_id(0).subsystem(0),
        }));
    future_2.wait();
    ASSERT_TRUE(future_2.valid());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(future_1.get()->get_status(), rclcpp_action::GoalStatus::STATUS_ABORTED);
    ASSERT_TRUE(future_2.get()->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                future_2.get()->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING);
}

}  // namespace actions