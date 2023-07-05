#pragma once
#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/world_state.hpp>

#include "position.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"
#include <rj_msgs/msg/robot_state.hpp>


namespace strategy {
class RobotMarker : public RoleInterface {
private:
    const double factor = 0.5;
    u_int8_t target_robot_id = 0;

public:
    RobotMarker(u_int8_t robot_id);
    ~RobotMarker() = default;
    virtual std::optional<RobotIntent> get_task(RobotIntent intent,
                                                const WorldState* const world_state, FieldDimensions field_dimensions) override;
};
}  // namespace strategy