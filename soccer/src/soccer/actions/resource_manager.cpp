#include "resource_manager.hpp"

namespace actions {

ResourceManager::ResourceManager() : rclcpp::Node("resource_manager_node") {
    resource_server_ = rclcpp_action::create_server<rj_msgs::action::UseResource>(
        this, "resource_manager",
        [this](const auto& uuid, auto goal) { return handle_goal(uuid, goal); },
        [this](auto goal) { return handle_cancel(goal); },
        [this](auto goal) { handle_accepted(goal); });
    release_timer_ =
        create_wall_timer(std::chrono::milliseconds(1), [this]() { release_callback(); });
}

rclcpp_action::GoalResponse ResourceManager::handle_goal(
    const rclcpp_action::GoalUUID& uuid, rj_msgs::action::UseResource::Goal::ConstSharedPtr goal) {
    for (const auto& resource : goal->resources_requested) {
        if (!can_take_resource(rj_convert::convert_from_ros(resource), goal->priority)) {
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ResourceManager::handle_accepted(GoalHandle handle) {
    take_resources(rj_convert::convert_from_ros(handle->get_goal()->resources_requested), handle);
    SPDLOG_INFO("Accepted {}", rclcpp_action::to_string(handle->get_goal_id()));
    display();
}

rclcpp_action::CancelResponse ResourceManager::handle_cancel(GoalHandle handle) {
    SPDLOG_INFO("Releasing {} because of canceling", rclcpp_action::to_string(handle->get_goal_id()));
    release_resources(handle->get_goal_id());
    release_queue_.push_back(handle);
    return rclcpp_action::CancelResponse::ACCEPT;
}

bool ResourceManager::can_take_resource(const Resource& resource, int8_t priority) {
    auto elem = in_use_resources_.find(resource);
    if (elem == in_use_resources_.end()) {
        return true;
    }

    auto old_handle = elem->second;
    return priority >= old_handle->get_goal()->priority;
}

void ResourceManager::take_resources(std::vector<Resource> resources, GoalHandle handle) {
    for (const auto& resource : resources) {
        auto it = in_use_resources_.find(resource);
        if (it != in_use_resources_.end()) {
            // Evict and abort the old goal so we can use the new one.
            auto old_handle = it->second;
            SPDLOG_INFO("Releasing {} because we're taking {}", rclcpp_action::to_string(old_handle->get_goal_id()), rclcpp_action::to_string(handle->get_goal_id()));
            release_resources(old_handle->get_goal_id());
            release_queue_.push_back(old_handle);
        }
        in_use_resources_[resource] = handle;
    }
    resources_by_uuid_[handle->get_goal_id()] = std::make_pair(std::move(resources), handle);
}

void ResourceManager::release_callback() {
    while (!release_queue_.empty()) {
        auto handle = release_queue_.front();
        release_queue_.pop_front();
        handle->abort(std::make_shared<rj_msgs::action::UseResource::Result>());
        SPDLOG_INFO("Released {}", rclcpp_action::to_string(handle->get_goal_id()));
        display();
    }
}

void ResourceManager::release_resources(const rclcpp_action::GoalUUID& uuid) {
    auto resource_iter = resources_by_uuid_.find(uuid);
    if (resource_iter == resources_by_uuid_.end()) {
        throw std::runtime_error(fmt::format("Double-freeing resources (or freeing unallocated resources) for UUID {}", rclcpp_action::to_string(uuid)));
    }

    auto& [resources, handle] = resource_iter->second;

    for (const auto& resource : resources) {
        in_use_resources_.erase(resource);
    }

    resources_by_uuid_.erase(resource_iter);
}

void ResourceManager::display() {
    for (auto& entry : in_use_resources_) {
        const auto& [resource, handle] = entry;
        SPDLOG_INFO("Robot {} : {} -> {}", resource.robot_id, resource.subsystem,
                    rclcpp_action::to_string(handle->get_goal_id()));
    }

    for (auto& entry : resources_by_uuid_) {
        const auto& [uuid, res] = entry;
        const auto& [resources, handle] = res;

        std::stringstream resources_string;
        for (size_t i = 0; i < resources.size(); i++) {
            if (i != 0) {
                resources_string << ",";
            }
            resources_string << "Robot " << resources.at(i).robot_id << " : "
                             << static_cast<int>(resources.at(i).subsystem);
        }
        SPDLOG_INFO("Goal {} -> [{}]", rclcpp_action::to_string(uuid), resources_string.str());
    }
}

}  // namespace actions