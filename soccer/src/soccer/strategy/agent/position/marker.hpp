#pragma once
#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/robot_state.hpp>
#include <rj_msgs/msg/world_state.hpp>

#include "position.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {
class Marker : public RoleInterface {
private:
    const double factor = 0.25;

public:
    Marker();
    ~Marker() = default;
    virtual std::optional<RobotIntent> get_task(RobotIntent intent,
                                                const WorldState* const world_state,
                                                FieldDimensions field_dimensions) override;
};
}  // namespace strategy