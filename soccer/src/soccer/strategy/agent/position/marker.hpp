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
class Marker : public RoleInterface {
private:
    int target{-1};

    static constexpr double Y_BOUND{4.5};

public:
    Marker(u_int8_t robot_id);
    ~Marker() = default;
    std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* const world_state,
                                        FieldDimensions field_dimensions) override;

    int choose_target(WorldState* ws);
    int get_target();
    bool target_out_of_bounds(WorldState* ws);
};
}  // namespace strategy
