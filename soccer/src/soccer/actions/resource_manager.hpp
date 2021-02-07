#pragma once

#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/action/use_resource.hpp>
#include <rj_msgs/msg/resource.hpp>

#include "resource.hpp"

namespace actions {

/**
 * @brief An action server to keep track of which robots' subsystems are in use.
 *
 * @details A server for regular actions should call out to this server to take control of a
 * resource (robot/subsystem pair) for the duration of the time for which it is needed. For example,
 * the "move to point" action requires usage of a single robot's drivetrain.
 */
class ResourceManager : public rclcpp::Node {
public:
    using GoalHandle =
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rj_msgs::action::UseResource>>;
    using WeakGoalHandle =
        std::weak_ptr<rclcpp_action::ServerGoalHandle<rj_msgs::action::UseResource>>;

    ResourceManager();

    void shutdown() {
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        rj_msgs::action::UseResource::Goal::ConstSharedPtr goal);
    void handle_accepted(GoalHandle handle);
    rclcpp_action::CancelResponse handle_cancel(GoalHandle handle);

    bool can_take_resource(const Resource& resource, int8_t priority);
    void take_resources(std::vector<Resource> resources, GoalHandle handle);
    void release_resources(const rclcpp_action::GoalUUID& uuid);

    /**
     * @brief Release the handle for a canceled/aborted goal.
     * @details Calling the destructor for the ServerGoalHandle class in one of the callbacks
     * directly will cause deadlock, as the destructor needs to grab a mutex. This is a hack to get
     * around it - we add the pointers to be removed from this queue, and then clean them up
     * regularly to avoid leaking memory.
     */
    void release_callback();

    /**
     * @brief Display debug information for the server to the logger.
     * @details Currently displays contents of maps from resources to goals and UUIDs to resources.
     */
    void display();

    rclcpp_action::Server<rj_msgs::action::UseResource>::SharedPtr resource_server_;
    std::unordered_map<Resource, GoalHandle, ResourceHash> in_use_resources_;
    std::deque<GoalHandle> release_queue_;
    std::unordered_map<rclcpp_action::GoalUUID, std::pair<std::vector<Resource>, GoalHandle>>
        resources_by_uuid_;
    rclcpp::TimerBase::SharedPtr release_timer_;
};

}  // namespace actions