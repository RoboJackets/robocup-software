#pragma once
#include <rclcpp/rclcpp.hpp>

#include <rj_geometry/segment.hpp>
#include <rj_msgs/msg/robot_state.hpp>
#include <rj_msgs/msg/world_state.hpp>

#include "position.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {

/**
 * Represents a Marker, a defensive role that targets an enemy robot
 * and follows it around the field while it is on our side, blocking passes.
 */
class BallMarker : public RoleInterface {
private:
    int target_{-1};

    // Calculated from field dimensions - Prevent the marker from
    // marking any enemies that are out of range; e. g. on the other
    // side of the field or the sidelines.
    float y_bound{FieldDimensions::kDefaultDimensions.length() / 2};
    float marker_follow_cutoff{FieldDimensions::kDefaultDimensions.width() / 2};

public:
    BallMarker(FieldDimensions field_dimensions);
    ~BallMarker() = default;
    BallMarker(const BallMarker& other) = default;
    BallMarker(BallMarker&& other) = default;
    BallMarker& operator=(const BallMarker& other) = default;
    BallMarker& operator=(BallMarker&& other) = default;

    std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* world_state,
                                        FieldDimensions field_dimensions) override;

    void choose_target(const WorldState* ws);
    int get_target();
    bool target_out_of_bounds(const WorldState* ws);
};
}  // namespace strategy
