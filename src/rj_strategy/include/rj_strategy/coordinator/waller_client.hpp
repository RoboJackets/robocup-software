#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_msgs/msg/waller.hpp>
#include <rj_msgs/srv/waller.hpp>

#include "rj_strategy/coordinator/waller.hpp"

namespace strategy {

/**
 * @brief Client for interacting with the Waller coordinator.
 *
 * Manages membership in the waller group and tracks the currently waller list.
 */
class WallerClient {
public:
    struct Result {
        bool success{false};
    };

    using StatusCallback = std::function<void(Result)>;

    explicit WallerClient(rclcpp::Node::SharedPtr node, uint8_t robot_id);
    ~WallerClient() = default;
    WallerClient(const WallerClient&) = delete;
    WallerClient& operator=(const WallerClient&) = delete;
    WallerClient(WallerClient&&) = delete;
    WallerClient& operator=(WallerClient&&) = delete;

    /**
     * @brief Join the waller group.
     * @param callback Called with current membership status after attempt to join.
     */
    void join_group(StatusCallback callback = nullptr);

    /**
     * @brief Leave the waller group.
     * @param callback Called with current membership status after attempt to leave.
     */
    void leave_group(StatusCallback callback = nullptr);

    /**
     * @brief Check if this robot is a member of the waller group.
     */
    [[nodiscard]] bool am_i_member() const;

    /**
     * @brief Get the target walling point
     * @return target walling point of this robot.
     */
    [[nodiscard]] std::optional<rj_geometry::Point> get_walling_point(
        const WorldState* world_state, FieldDimensions field_dimensions) const;

private:
    rclcpp::Node::SharedPtr node_;
    const uint8_t robot_id_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members) -- class
                              // isn't move/copy-able anyway
    rclcpp::Client<rj_msgs::srv::Waller>::SharedPtr client_;
    rclcpp::Subscription<rj_msgs::msg::Waller>::SharedPtr subscription_;

    bool am_i_member_ = false;
    bool request_pending_ = false;
    std::array<uint8_t, kNumShells> walling_robots_;
    int num_wallers_ = 0;

    static constexpr double kRobotDiameterMultiplier = 1.5;

    struct WallerGeometry {
        double min_wall_radius;
        double wall_spacing;
        rj_geometry::Point goal_pos;
        rj_geometry::Point ball_pos;
        rj_geometry::Point robot_pos;
    };

    WallerGeometry calculate_wall_geometry(const WorldState* world_state,
                                           FieldDimensions dimensions) const;
    rj_geometry::Point get_target_position(WallerGeometry& waller_geometry, long waller_pos) const;
    std::optional<uint8_t> get_parent_id(WallerGeometry& waller_geometry,
                                         rj_geometry::Point target_point, long waller_pos) const;
    rj_geometry::Point get_target_position_with_parent(WallerGeometry& waller_geometry,
                                                       rj_geometry::Point target_point,
                                                       rj_geometry::Point parent_point) const;
};

}  // namespace strategy