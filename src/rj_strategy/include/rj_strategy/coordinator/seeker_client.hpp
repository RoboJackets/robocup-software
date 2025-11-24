#pragma once

#include <functional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/seeker_coordinator.hpp>
#include <rj_msgs/srv/seeker_coordinator.hpp>

#include "rj_strategy/coordinator/seeker_coordinator.hpp"

namespace strategy {

/**
 * @brief Client for interacting with the Seeker coordinator.
 *
 * Manages membership in the Seeker group and the seekers' next target positions.
 */
class SeekerClient {
public:

    struct Result {
        bool am_i_member{false};  // Whether this robot is currently a member of the seeker group.
    };
    
    using StatusCallback = std::function<void(Result)>;

    explicit SeekerClient(rclcpp::Node::SharedPtr node, uint8_t robot_id);
    ~SeekerClient() = default;
    SeekerClient(const SeekerClient&) = delete;
    SeekerClient& operator=(const SeekerClient&) = delete;
    SeekerClient(SeekerClient&&) = delete;
    SeekerClient& operator=(SeekerClient&&) = delete;

    /**
     * @brief Join the seeker group. or poll for a new target position.
     * @param callback Called with current membership status after attempt to join.
     */
    void poll_for_target(StatusCallback callback = nullptr);

    /**
     * @brief Leave the seeker group.
     * @param callback Called with current membership status after attempt to leave.
     */
    void leave_group(StatusCallback callback = nullptr);

    /**
     * @brief Check if this robot is a member of the seeker group.
     */
    [[nodiscard]] bool am_i_member() const;

    /**
     * @brief Get the current target position.
     * @return rj_geometry::Point to target, or Point{-1,-1} if none selected.
     */
    [[nodiscard]] rj_geometry::Point selected_target() const;


private:
    rclcpp::Node::SharedPtr node_;
    const uint8_t robot_id_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members) -- class
                              // isn't move/copy-able anyway
    rclcpp::Client<rj_msgs::srv::SeekerCoordinator>::SharedPtr client_;
    rclcpp::Subscription<rj_msgs::msg::SeekerCoordinator>::SharedPtr subscription_;

    bool am_i_member_{false};
    rj_geometry::Point selected_target_ {-1, -1};
};

}  // namespace strategy
