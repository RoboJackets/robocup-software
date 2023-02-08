#include "blocker.hpp"


namespace strategy {
    std::optional<RobotIntent> Blocker::get_task(RobotIntent intent, const WorldState* const world_state) {
        //RobotState& robot {world_state->get_robot(true, intent.robot_id)};
        //BallState& ball {world_state->ball};

        // face ball
        planning::PathTargetFaceOption face_option = planning::FaceBall{};
        // avoid ball
        bool ignore_ball = false;
        //planning::LinearMotionInstant goal{world_state->ball.position, rj_geometry::Point{0.0, 0.0}};
        //intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
        //intent.motion_command_name = "blocker_target";
        return intent;
    }
}