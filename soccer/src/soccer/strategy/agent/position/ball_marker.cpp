#include "ball_marker.hpp"

namespace strategy {
BallMarker::BallMarker(FieldDimensions field_dimensions) {
    this->y_bound = field_dimensions.length() / 2;
    this->marker_follow_cutoff = field_dimensions.width() / 2;
}

std::optional<RobotIntent> BallMarker::get_task(RobotIntent intent, const WorldState* world_state,
                                            [[maybe_unused]] FieldDimensions field_dimensions) {
    this->y_bound = field_dimensions.length() / 2;
    this->marker_follow_cutoff = field_dimensions.width() / 2;

    rj_geometry::Point targetPoint = world_state->get_robot(false, target_).pose.position();
    rj_geometry::Point goalPoint = field_dimensions.our_goal_loc();
    rj_geometry::Point targetToBall = (goalPoint - targetPoint).normalized(0.25f);
    planning::LinearMotionInstant goal{targetPoint + targetToBall, rj_geometry::Point{0.0, 0.0}};
    intent.motion_command =
        planning::MotionCommand{"path_target", goal, planning::FaceBall{}, true};
    return intent;
}

void BallMarker::choose_target(const WorldState* world_state) {
    // TODO: (James Vogt, github: jvogt23)
    // If we ever use multiple Markers, they should choose different
    // robots to track from each other. Logic for this operation must be
    // added because multiple markers currently mark the same robot.
    float min_dist = 100000;
    for (int i = 0; i < kNumShells; i++) {
        rj_geometry::Point theirRobotPos = world_state->get_robot(false, i).pose.position();
        rj_geometry::Point ballPoint = world_state->ball.position;
        float dist = theirRobotPos.dist_to(ballPoint);
        if (dist < min_dist) {
            min_dist = dist;
            target_ = i;
        }
    }
    // target_ = 2;
}

bool BallMarker::target_out_of_bounds(const WorldState* ws) {
    if (target_ == -1) return true;
    if (ws->get_robot(false, target_).pose.position().y() > y_bound) {
        target_ = -1;
        return true;
    }
    return false;
}

int BallMarker::get_target() { return target_; }
}  // namespace strategy