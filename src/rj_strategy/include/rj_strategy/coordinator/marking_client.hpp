#pragma once

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/marking.hpp>
#include <rj_msgs/srv/marking.hpp>
#include "rj_constants/constants.hpp"

namespace strategy {

/**
 * @brief Client for interacting with the KickerPicker coordinator.
 *
 * Manages membership in the kicker group and tracks the currently selected kicker.
 */
class MarkingClient {
public:
    static constexpr uint8_t kInvalidRobotId = kNumShells;
    struct Result {
        bool am_i_member{false};  // Whether this robot is currently a member of the kicker group.
        std::optional<bool> am_i_marking{false};
        std::optional<uint8_t> who_i_am_marking{kInvalidRobotId};  // ID of Kicker id
    };

    using StatusCallback = std::function<void(Result)>;

    explicit MarkingClient(rclcpp::Node::SharedPtr node, uint8_t robot_id);
    ~MarkingClient() = default;
    MarkingClient(const MarkingClient&) = delete;
    MarkingClient& operator=(const MarkingClient&) = delete;
    MarkingClient(MarkingClient&&) = delete;
    MarkingClient& operator=(MarkingClient&&) = delete;

    /**
     * @brief Join the marking group.
     * @param callback Called with current membership status after attempt to join.
     */
    void join_group(StatusCallback callback = nullptr);

    /**
     * @brief Leave the marking group.
     * @param callback Called with current membership status after attempt to leave.
     */
    void leave_group(StatusCallback callback = nullptr);

    /**
     * @brief Check if this robot is a member of the marker group.
     */
    [[nodiscard]] bool am_i_member() const;

    /**
     * @brief Get the currently selected enemey robot id marking.
     * @return robot ID of selected robot, or kInvalidRobotId if none selected.
     */
    [[nodiscard]] uint8_t who_am_i_marking() const;

    /**
     * @brief Check if this robot is currently marking.
     */
    [[nodiscard]] bool am_i_marking() const;

private:
    rclcpp::Node::SharedPtr node_;
    const uint8_t robot_id_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members) -- class
                              // isn't move/copy-able anyway
    rclcpp::Client<rj_msgs::srv::Marking>::SharedPtr client_;
    rclcpp::Subscription<rj_msgs::msg::Marking>::SharedPtr subscription_;
    bool am_i_member_{false};
    bool am_i_marking_{false};
    uint8_t selected_robot_marking_id_{kInvalidRobotId};
};

}  // namespace strategy
