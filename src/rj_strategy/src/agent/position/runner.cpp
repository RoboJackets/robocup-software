#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {
    Runner::Runner(int r_id) : Offense{r_id} {
        position_name_ = "Runner";
    }
    Runner::Runner(const Position& other) : Offense{other}{

    }
    std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent){
        
        switch (current_state_){
            case CENTER:
                {
                auto motion_command = planning::MotionCommand{
                    "path_target",
                    planning::LinearMotionInstant{
                        rj_geometry::Point{1.33,2.29},
                        rj_geometry::Point{0,0},
                    },
                    planning::FaceTarget(), true};

                intent.motion_command = motion_command;
                return intent;
                }
            case DRIVING_TO_TOP_LEFT:
            
                {auto motion_command = planning::MotionCommand{
                    "path_target",
                    planning::LinearMotionInstant{
                        rj_geometry::Point{1.33-1,2.29},
                        rj_geometry::Point{0,0},
                    },
                    planning::FaceTarget(), true};

                intent.motion_command = motion_command;
                return intent;}
            case DRIVING_TO_TOP_RIGHT:
                {auto motion_command = planning::MotionCommand{
                    "path_target",
                    planning::LinearMotionInstant{
                        rj_geometry::Point{1.33-1,2.29+1},
                        rj_geometry::Point{0,0},
                    },
                    planning::FaceTarget(), true};

                intent.motion_command = motion_command;
                return intent;}
            case DRIVING_TO_BOTTOM_RIGHT:
                {auto motion_command = planning::MotionCommand{
                    "path_target",
                    planning::LinearMotionInstant{
                        rj_geometry::Point{1.33,2.29+1},
                        rj_geometry::Point{0,0},
                    },
                    planning::FaceTarget(), true};

                intent.motion_command = motion_command;
                return intent;}
            case DRIVING_TO_BOTTOM_LEFT:
                {auto motion_command = planning::MotionCommand{
                    "path_target",
                    planning::LinearMotionInstant{
                        rj_geometry::Point{1.33,2.29},
                        rj_geometry::Point{0,0},
                    },
                    planning::FaceTarget(), true};

                intent.motion_command = motion_command;
                return intent;}
            default:
                return intent;

            
        }
    }
    
    Runner::State Runner::next_state(){
        switch (current_state_){
            case CENTER:
                if ( check_is_done())
                return DRIVING_TO_TOP_LEFT;
                else 
                return CENTER;
            case TOP_LEFT:
                return DRIVING_TO_TOP_RIGHT;
            case BOTTOM_LEFT:
                return DRIVING_TO_TOP_LEFT;
            case BOTTOM_RIGHT:
                return DRIVING_TO_BOTTOM_LEFT;
            case TOP_RIGHT:
                return DRIVING_TO_BOTTOM_RIGHT;
            case DRIVING_TO_BOTTOM_LEFT:
                if (check_is_done()){
                    return BOTTOM_LEFT;
                }
            case DRIVING_TO_BOTTOM_RIGHT:
                if (check_is_done()){
                    return BOTTOM_RIGHT;
                }
            case DRIVING_TO_TOP_LEFT:
                if (check_is_done()){
                    return TOP_LEFT;
                }
            case DRIVING_TO_TOP_RIGHT:
                if (check_is_done()){
                    return TOP_RIGHT;
                }
                return current_state_;             
        }
    }
    std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
        
        State new_state = next_state();
        std::optional<RobotIntent> newIntent = state_to_task(intent);
        current_state_ = new_state;
        SPDLOG_INFO("Runner state: {}", state_to_name(current_state_));
        
        return newIntent;
    }
}