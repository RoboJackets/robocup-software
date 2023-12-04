


#include "runner.hpp"
//ask how to configure c++ extension
namespace strategy {

Runner::Runner(int r_id) : Position(r_id) { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    State next_state = current_state_;

    switch (current_state_) {
        case LEFT:
            if (check_is_done()) {
                next_state = DOWN;
            }
            break;
        case DOWN:
            if (check_is_done()) {
                next_state = RIGHT;
            }
            break;
        case RIGHT:
            if (check_is_done()) {
                next_state = UP;
            }
        case UP:
            if (check_is_done()) {
                next_state = LEFT;
            }
    }

    return next_state;
}
// (-2,1) topleft (2,1) (2,7) (-2,7)

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
    
    if (current_state_ == LEFT) {
        auto motion_instance =
                planning::LinearMotionInstant{rj_geometry::Point{2, 1}, {0,0}};
        auto face_ball_cmd = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}};
        intent.motion_command = face_ball_cmd;
        return intent;
    } else if (current_state_ == DOWN) {
        auto motion_instance =
                planning::LinearMotionInstant{rj_geometry::Point{2, 7}, {0,0}};
        auto face_ball_cmd = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}};
        intent.motion_command = face_ball_cmd;
        return intent;
    } else if (current_state_ == RIGHT) {
        auto motion_instance =
                planning::LinearMotionInstant{rj_geometry::Point{-2, 7}, {0,0}};
        auto face_ball_cmd = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}};
        intent.motion_command = face_ball_cmd;
        return intent;
    } else if (current_state_ == UP) {
        auto motion_instance =
                planning::LinearMotionInstant{rj_geometry::Point{-2, 1}, {0,0}};
        auto face_ball_cmd = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}};
        intent.motion_command = face_ball_cmd;
        return intent;
    }
    return std::nullopt;
}

void Runner::receive_communication_response(communication::AgentPosResponseWrapper response) {
    // Call to super
    Position::receive_communication_response(response);

    // Handle join wall response
    // if (const communication::JoinWallRequest* join_request =
    //         std::get_if<communication::JoinWallRequest>(&response.associated_request)) {
    //     for (communication::AgentResponseVariant response : response.responses) {
    //         if (const communication::JoinWallResponse* join_response =
    //                 std::get_if<communication::JoinWallResponse>(&response)) {
    //             handle_join_wall_response(*join_response);
    //         }
    //     }
    // }
}

communication::PosAgentResponseWrapper Runner::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    // Call to super
    communication::PosAgentResponseWrapper response =
        Position::receive_communication_request(request);

    // Handle join and leave wall request
    // if (const communication::JoinWallRequest* join_request =
    //         std::get_if<communication::JoinWallRequest>(&request.request)) {
    //     response.response = handle_join_wall_request(*join_request);
    // } else if (const communication::LeaveWallRequest* leave_request =
    //                std::get_if<communication::LeaveWallRequest>(&request.request)) {
    //     response.response = handle_leave_wall_request(*leave_request);
    // }

    // Return the response
    return response;
}






void Runner::derived_acknowledge_pass() { }

void Runner::derived_pass_ball() { }

void Runner::derived_acknowledge_ball_in_transit() {
}


void Runner::die() {
   
}

void Runner::revive() { }

}  // namespace strategy
