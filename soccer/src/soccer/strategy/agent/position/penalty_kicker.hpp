#pragma once

#include <rclcpp/rclcpp.hpp>

#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {

class PenaltyKicker : public Position {
public:
    PenaltyKicker(int r_id);
    ~PenaltyKicker() = default;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

}  // namespace strategy