#include <functional>
#include <memory>
#include <thread>

// ros2 action includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// rj includes
#include <rj_common/utils.hpp>
#include <rj_msgs/action/move.hpp>

namespace server {
class MoveActionServer : public rclcpp::Node {
    using Move = rj_msgs::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    explicit MoveActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("move_action_server", options) {
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<Move>(
            this, "move", std::bind(&MoveActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveActionServer::handle_cancel, this, _1),
            std::bind(&MoveActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<Move>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const Move::Goal> goal) {
        std::cout << "handle goal reached" << std::endl;
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
        std::cout << "cancel reached" << std::endl;
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
        using namespace std::placeholders;
        std::cout << "accepted reached" << std::endl;
        std::thread{std::bind(&MoveActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
        std::cout << "executing" << std::endl;
        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<Move::Feedback>();
        auto result = std::make_shared<Move::Result>();

        result->is_done = true;
        goal_handle->succeed(result);
    }
}; // class 
}  // namespace server

// TODO : Use this?
// RCLCPP_COMPONENTS_REGISTER_NODE(rj_robocup::MoveActionServer)
