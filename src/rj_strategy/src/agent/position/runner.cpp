#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

    Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

    Runner::Runner(Position&& other) : Position{std::move(other)} {
        position_name_ = "Runner";
    }

    static const rj_geometry::Point firstVertex{2.0, 2.0};
    static const rj_geometry::Point secondVertex{-2.0, 2.0};
    static const rj_geometry::Point thirdVertex{-2.0, 6.0};
    static const rj_geometry::Point fourthVertex{2.0, 6.0};

    std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
        //first get the proper state and set current state to be that
        current_state_ = next_state();

        return state_to_task(intent);
    }

    Runner::State Runner::next_state() {
        //handle transitions between states
        switch (current_state_) {
            case DEFAULT: {
                return FIRST_SIDE;
            }

            case FIRST_SIDE: {
                return check_is_done() ? SECOND_SIDE : FIRST_SIDE;
            }

            case SECOND_SIDE: {
                return check_is_done() ? THIRD_SIDE : SECOND_SIDE;
            }

            case THIRD_SIDE: {
                return check_is_done() ? FOURTH_SIDE : THIRD_SIDE;
            }

            case FOURTH_SIDE: {
                return check_is_done() ? FIRST_SIDE : FOURTH_SIDE;
            }
        }

        return DEFAULT;
    }

    std::string Runner::get_current_state() {
        return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
    }

    std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
        switch (current_state_) {
            case DEFAULT: {
                //go to first side
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{firstVertex}
                };
                return intent;
            }

            case FIRST_SIDE: {
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{firstVertex}
                };
                return intent;
            }

            case SECOND_SIDE: {
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{secondVertex}
                };
                return intent;
            }

            case THIRD_SIDE: {
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{thirdVertex}
                };
                return intent;
            }

            case FOURTH_SIDE: {
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{fourthVertex}
                };
                return intent;
            }
        }

        return nullopt;
    }
}