#include "runner.hpp"

#include <spdlog/spdlog.h>

namespace strategy {
    Runner::Runner (int r_id) : Position(r_id){
        position_name_ = "Runner";
        latest_state_ = UP;
    }

    std::optional<RobotIntent> Runner::derived_get_task(RobotIntent rIntent) {
        latest_state_ = update_state();
        return state_to_task(rIntent);
    }

    Runner::State Runner::update_state() {
        WorldState* world_state = this->world_state();
        rj_geometry::Point curr_pos = world_state->get_robot(true, robot_id_).pose.position();
        if(this->latest_state_ == UP) {
            if(curr_pos.dist_to(this->point_up) <= 0.1)
            {
                return LEFT;
            }
            else {
                return UP;
            }
        }
        if(this->latest_state_ == LEFT) {
            if(curr_pos.dist_to(this->point_left) <= 0.1)
            {
                return DOWN;
            }
            else {
                return LEFT;
            }
        }

        if(this->latest_state_ == DOWN) {
            if(curr_pos.dist_to(this->point_down) <= 0.1)
            {
                return RIGHT;
            }
            else {
                return DOWN;
            }
        }
        if(this->latest_state_ == RIGHT) {
            if(curr_pos.dist_to(this->point_right) <= 0.1)
            {
                return UP;
            }
            else {
                return RIGHT;
            }
        }
    }


    std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
        rj_geometry::Point target_vel{0.0, 0.0};
        planning::PathTargetFaceOption face_option{planning::FaceTarget{}};
        planning::LinearMotionInstant target;
        if (this->latest_state_ == UP) {
            target = planning::LinearMotionInstant{point_up, target_vel};
        }
        else if (this->latest_state_ == LEFT) {
            target = planning::LinearMotionInstant{point_left, target_vel};
        }
        else if (this->latest_state_ == DOWN) {
            target = planning::LinearMotionInstant{point_down, target_vel};
        }
        else if (this->latest_state_ == RIGHT) {
            target = planning::LinearMotionInstant{point_right, target_vel};
        }
        bool ignore_ball{true};
        intent.motion_command =
            planning::MotionCommand{"path_target", target, face_option, ignore_ball};
        return intent;
    }
}