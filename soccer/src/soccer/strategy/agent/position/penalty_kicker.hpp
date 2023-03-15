#include "role_interface.hpp"

namespace strategy {

class PenaltyKicker : public RoleInterface {
public:
    PenaltyKicker();
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