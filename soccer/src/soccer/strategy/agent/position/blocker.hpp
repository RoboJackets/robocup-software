#pragma once
#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/world_state.hpp>

#include "position.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {
class Blocker : public RoleInterface {
private:
    const double factor{};

public:
    Blocker(double factor = 0.75);
    virtual std::optional<RobotIntent> get_task(RobotIntent intent,
                                                const WorldState* const world_state) override;
};
}  // namespace strategy