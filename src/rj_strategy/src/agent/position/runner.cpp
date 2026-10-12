#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

    Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

    Runner::Runner(Position&& other) : Position{std::move(other)} {
        position_name_ = "Runner";
    }

    static const rj_geometry::Point firstVertex{1.0, 1.0};
    static const rj_geometry::Point secondVertex{1.0, 1.0};
    static const rj_geometry::Point thirdVertex{1.0, 1.0};
    static const rj_geometry::Point fourthVertex{1.0, 1.0};

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) {
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
                if(at_first_vertex()) {
                    return SECOND_SIDE;
                }
            }

            case SECOND_SIDE: {
                if(at_second_vertex()) {
                    return THIRD_SIDE;
                }
            }

            case THIRD_SIDE: {
                if(at_third_vertex()) {
                    return FOURTH_SIDE;
                }
            }

            case FOURTH_SIDE: {
                if(at_fourth_vertex()) {
                    return FIRST_SIDE;
                }
            }
        }
    }

    std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
        switch (current_state_) {
            case DEFAULT: {
                //wait for state to be switched to first side
                intent.motion_command = planning::MotionCommand{};
                return intent;
            }

            case FIRST_SIDE: {
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{firstVertex}
                };
            }

            case SECOND_SIDE: {
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{secondVertex}
                };
            }

            case THIRD_SIDE: {
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{thirdVertex}
                };
            }

            case FOURTH_SIDE: {
                intent.motion_command = planning::MotionCommand{
                    "path_target", planning::LinearMotionInstant{fourthVertex}
                };
            }
        }
    }

    bool Runner::at_first_vertex() {
        auto current_position = last_world_state_->get_robot(true, robot_id_).pose.position();
        return current_position == firstVertex;
    }

    bool Runner::at_second_vertex() {
        auto current_position = last_world_state_->get_robot(true, robot_id_).pose.position();
        return current_position == secondVertex;
    }

    bool Runner::at_third_vertex() {
        auto current_position = last_world_state_->get_robot(true, robot_id_).pose.position();
        return current_position == thirdVertex;
    }

    bool Runner::at_fourth_vertex() {
        auto current_position = last_world_state_->get_robot(true, robot_id_).pose.position();
        return current_position == fourthVertex;
    }
}