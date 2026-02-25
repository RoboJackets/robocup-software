#include "rj_strategy/agent/position/dribbler.hpp"
#include "rj_geometry/util.hpp"
#include "spdlog/spdlog.h"
#include <limits>
#include <algorithm>

namespace strategy {

Dribbler::Dribbler(int r_id) : Position(r_id, "Dribbler") {}

void Dribbler::set_dribble_target(rj_geometry::Point target) {
    target_point_ = target;
}

void Dribbler::set_dribbler_speed(float speed) {
    // Clamp speed to valid range [0.0, 1.0]
    dribbler_speed_ = std::clamp(speed, 0.0f, 1.0f);
}

std::string Dribbler::get_current_state() {
    if (assert_world_state_valid() && has_ball()) {
        return "Dribbling";
    } else {
        return "Collecting";
    }
}

communication::PosAgentResponseWrapper Dribbler::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    auto comm_response = Position::receive_communication_request(request);

    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        if (!has_ball()) {
            auto response = receive_pass_request(*pass_request);
            response.direct_open = true;
            comm_response.response = response;
            SPDLOG_DEBUG("Dribbler {}: accepting pass request from robot {}", 
                         robot_id_, pass_request->from_robot_id);
        }
    }
    return comm_response;
}

std::optional<RobotIntent> Dribbler::derived_get_task(RobotIntent intent) {
    if (!assert_world_state_valid()) {
        SPDLOG_WARN("Dribbler {}: world state invalid, returning empty command.", robot_id_);
        intent.motion_command = planning::MotionCommand{};
        intent.dribbler_mode = RobotIntent::DribblerMode::OFF;
        return intent;
    }

    const auto& robot = last_world_state_->get_robot(true, robot_id_);
    if (!robot.visible) {
        SPDLOG_WARN("Dribbler {}: robot not visible, disabling dribbler.", robot_id_);
        intent.motion_command = planning::MotionCommand{};
        intent.dribbler_mode = RobotIntent::DribblerMode::OFF;
        return intent;
    }

    // Check if game is halted - don't dribble during pause/halt
    if (current_play_state_.is_halt()) {
        SPDLOG_DEBUG("Dribbler {}: game halted, standing down.", robot_id_);
        intent.dribbler_mode = RobotIntent::DribblerMode::OFF;
        intent.motion_command = planning::MotionCommand{};
        return intent;
    }

    rj_geometry::Point ball_position = last_world_state_->ball.position;

    if (has_ball()) {
        // Use configurable target point
        rj_geometry::Point ball_to_target = target_point_ - ball_position;
        double dist_to_target = ball_to_target.mag();

        if (dist_to_target > 0.001) {
            ball_to_target = ball_to_target / dist_to_target; // normalize
        }

        double current_dist = distance_to_ball();
        
        // If ball drifts too far while dribbling, switch to collection
        if (current_dist > kCollectionDistance) {
            intent.motion_command = planning::MotionCommand{"collect"};
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;  // Keep dribbler on while collecting
        } else {
            // Lookahead toward target with reduced correction for better ball contact
            double lookahead_distance = std::min(1.0, std::max(0.4, dist_to_target * 0.4));

            // Smaller ball-correction vector to avoid drifting away from ball
            rj_geometry::Point robot_to_ball = ball_position - robot.pose.position();
            double correction_scale = 0.1; // Reduced from 0.2 to 10% for better contact
            rj_geometry::Point correction = robot_to_ball * correction_scale;

            rj_geometry::Point target_ahead = ball_position + ball_to_target * lookahead_distance + correction;

            planning::LinearMotionInstant target{target_ahead, rj_geometry::Point{0,0}};
            planning::MotionCommand cmd;
            cmd.name = "path_target";
            cmd.target = target;
            cmd.face_option = planning::FaceBall{};
            cmd.ignore_ball = true;

            intent.motion_command = cmd;

            SPDLOG_DEBUG("Dribbler {}: smooth dribble, ball at ({:.3f},{:.3f}), target ahead ({:.3f},{:.3f}), distance={:.3f}m, lookahead={:.3f}",
                         robot_id_, ball_position.x(), ball_position.y(),
                         target_ahead.x(), target_ahead.y(), current_dist, lookahead_distance);
            
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
        }
        
        intent.is_active = true;
        had_ball_last_cycle_ = true;
        return intent;
    } else {
        // Move to collect the ball with dribbler ON to grab it
        intent.motion_command = planning::MotionCommand{"collect"};
        intent.dribbler_mode = RobotIntent::DribblerMode::ON;  // Motor ON while collecting!
        intent.is_active = true;

        SPDLOG_DEBUG("Dribbler {}: collecting ball, distance={:.3f}m", 
                     robot_id_, distance_to_ball());
        
        had_ball_last_cycle_ = false;
        return intent;
    }
}

bool Dribbler::has_ball() const {
    if (last_world_state_ == nullptr) return false;
    
    double dist = distance_to_ball();
    
    // Use hysteresis to avoid rapid switching between having/not having ball
    if (had_ball_last_cycle_) {
        // If we had the ball, need more distance before losing it
        return dist < kReacquisitionDistance;
    } else {
        // If we didn't have it, need to be closer to claim we have it
        return dist < kOwnBallRadius;
    }
}

bool Dribbler::can_collect_ball() const {
    if (last_world_state_ == nullptr) return false;
    // Robot can attempt collection if ball is within collection distance
    return distance_to_ball() < kCollectionDistance;
}

double Dribbler::distance_to_ball() const {
    if (last_world_state_ == nullptr) return std::numeric_limits<double>::max();

    rj_geometry::Point robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = last_world_state_->ball.position;

    return robot_position.dist_to(ball_position);
}

}  // namespace strategy
