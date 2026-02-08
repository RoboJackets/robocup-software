#pragma once

#include <functional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/seeking_coordinator.hpp>
#include <rj_msgs/srv/seeking_coordinator.hpp>

#include "rj_strategy/coordinator/seeking_coordinator.hpp"

namespace strategy {

/**
 * @brief Client for interacting with the Seeker coordinator.
 *
 * Manages membership in the Seeker group and the seekers' next target positions.
 */
class SeekingClient {
public:
    struct Result {
        bool am_i_member{false};  // Whether this robot is currently a member of the seeker group.
    };

    using StatusCallback = std::function<void(Result)>;

    explicit SeekingClient(rclcpp::Node::SharedPtr node, uint8_t robot_id);
    ~SeekingClient() = default;
    SeekingClient(const SeekingClient&) = delete;
    SeekingClient& operator=(const SeekingClient&) = delete;
    SeekingClient(SeekingClient&&) = delete;
    SeekingClient& operator=(SeekingClient&&) = delete;

    /**
     * @brief Join the seeker group.
     * @param callback Called with current membership status after attempt to join.
     */
    void join_group(StatusCallback callback = nullptr);

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
    [[nodiscard]] std::shared_ptr<rj_geometry::Point> selected_target() const;

private:
    rclcpp::Node::SharedPtr node_;
    const uint8_t robot_id_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members) -- class
                              // isn't move/copy-able anyway
    rclcpp::Client<rj_msgs::srv::SeekingCoordinator>::SharedPtr client_;
    rclcpp::Subscription<rj_msgs::msg::SeekingCoordinator>::SharedPtr subscription_;

    bool am_i_member_{false};
    std::shared_ptr<rj_geometry::Point> selected_target_{nullptr};
};

}  // namespace strategy
