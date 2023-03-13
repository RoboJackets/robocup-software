#pragma once
#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/world_state.hpp>

#include "position.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {
class Marker : public RoleInterface {
private:
    const double factor{};

public:
    Marker(double factor = 0.75);
    virtual std::optional<RobotIntent> get_task(RobotIntent intent,
                                                const WorldState* const world_state) override;
    std::optional<RobotIntent> get_point_task(RobotIntent intent, const rj_geometry::Point& point1,
                                              const rj_geometry::Point& point2);
};
}  // namespace strategy