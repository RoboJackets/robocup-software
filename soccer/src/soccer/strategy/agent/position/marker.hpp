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
    int target_{-1};

    static constexpr double kYBound{4.5};
    //Constant - Avoid marking robots that are right on the edge of the field.
    static constexpr double kMarkerFollowCutoff {2.5};

public:
    Marker();
    ~Marker() = default;
    Marker(const Marker& other) = default;
    Marker(Marker&& other) = default;
    Marker& operator=(const Marker& other) = default;
    Marker& operator=(Marker&& other) = default;

    std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* world_state,
                                        FieldDimensions field_dimensions) override;

    void choose_target(const WorldState* ws);
    int get_target();
    bool target_out_of_bounds(const WorldState* ws);
};
}  // namespace strategy
