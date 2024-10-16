#include "zoner.hpp"

namespace strategy {

Zoner::Zoner(int r_id) : Position(r_id, "Zoner") {}

Zoner::Zoner(const Position& other) : Position{other} { position_name_ = "Zoner"; }

std::optional<RobotIntent> Zoner::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();
    // waller_id_ = get_waller_id();
    return state_to_task(intent);
}

Zoner::State Zoner::next_state() {
    rj_geometry::Point ball_pos = last_world_state_->ball.position;
    rj_geometry::Segment block_path{ball_pos, field_dimensions_.our_goal_loc()};

    double min_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(kNumShells); i++) {
        // TODO: Add solo offense here as well
        if (i == robot_id_ || i == goalie_id_) {
            continue;
        }
        if (alive_robots_[i]) {
            RobotState robot = last_world_state_->get_robot(true, i);
            double dist = block_path.dist_to(robot.pose.position());
            min_dist = std::min(min_dist, dist);
        }
    }

    if (min_dist < (2 * kRobotRadius)) {
        return ZONE;
    }
    return WALL;
}

std::optional<RobotIntent> Zoner::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case WALL: {
            float box_w{field_dimensions_.penalty_long_dist()};
            float box_h{field_dimensions_.penalty_short_dist()};
            float line_w{field_dimensions_.line_width()};
            double min_wall_rad{
                (kRobotRadius * 4.0f) + line_w +
                hypot(static_cast<double>(box_w) / 2, static_cast<double>((box_h)))};
            min_wall_rad *= 1.5;

            rj_geometry::Point goal_pos = field_dimensions_.our_goal_loc();
            rj_geometry::Point ball_pos = last_world_state_->ball.position;

            rj_geometry::Point target_point =
                ((ball_pos - goal_pos).normalized(min_wall_rad)) + goal_pos;

            rj_geometry::Point target_vel{0.0, 0.0};

            // Face ball
            planning::PathTargetFaceOption face_option{planning::FaceBall{}};

            // Avoid ball
            bool ignore_ball{true};

            // Create Motion Command
            planning::LinearMotionInstant target{target_point, target_vel};
            intent.motion_command =
                planning::MotionCommand{"path_target", target, face_option, ignore_ball};
            return intent;
        }

        case ZONE: {
            // can change this point depending on zoner behavior
            rj_geometry::Point center{1.35, 1.5};
            rj_geometry::Point ball_pos = last_world_state_->ball.position;

            if (field_dimensions_.our_goal_loc().y() > 4.5) {
                center.y() = -1 * center.y();
            }
            if (ball_pos.x() > 0) {
                center.x() = -1 * center.x();
            }

            std::vector<rj_geometry::Point> opp_poses{};

            for (RobotState opp : last_world_state_->their_robots) {
                double dist = center.dist_to(opp.pose.position());
                if (dist < kZonerRadius) {
                    opp_poses.emplace_back(opp.pose.position());
                }
            }
            // SPDLOG_INFO("opp pose size: {}", opp_poses.size());

            rj_geometry::Point target_point{};
            if (opp_poses.size() >= 2) {
                // finding point that minimizes distance from opponent robots
                target_point = find_centroid(opp_poses);
            } else if (opp_poses.size() == 1) {
                // mark if just one robot
                double mark_rad = kRobotRadius * 4;
                target_point = (ball_pos - opp_poses[0]).normalized(mark_rad) + opp_poses[0];
            } else {
                target_point = center;
            }

            rj_geometry::Point target_vel{0.0, 0.0};

            // Face ball
            planning::PathTargetFaceOption face_option{planning::FaceBall{}};

            // Avoid ball
            bool ignore_ball{true};

            // Create Motion Command
            planning::LinearMotionInstant target{target_point, target_vel};
            intent.motion_command =
                planning::MotionCommand{"path_target", target, face_option, ignore_ball};
            return intent;
        }
    }
}

rj_geometry::Point Zoner::find_centroid(const std::vector<rj_geometry::Point> opp_poses) {
    rj_geometry::Point p{};

    for (rj_geometry::Point pos : opp_poses) {
        p += pos;
    }

    return p / static_cast<double>(opp_poses.size());
}

std::string Zoner::get_current_state() { return "Zoner"; }

}  // namespace strategy
