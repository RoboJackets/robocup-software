#pragma once

#include <rclcpp/rclcpp.hpp>

#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {

class PenaltyKicker : public RoleInterface {
public:
    PenaltyKicker() = default;
    ~PenaltyKicker() = default;

    /**
     * @brief  Returns a PenaltyKicker behavior for offensive robot that kicks the ball
     *
     * @param [RobotIntent intent] [current RobotIntent of the robot]
     * @return [RobotIntent with next target point for the robot]
     */
    std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* world_state,
                                        FieldDimensions field_dimensions) override;
};

}  // namespace strategy