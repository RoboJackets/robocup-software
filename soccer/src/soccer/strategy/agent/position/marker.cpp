#include "marker.hpp"

namespace strategy {
Marker::Marker(FieldDimensions field_dimensions) {
    this->y_bound = field_dimensions.length() / 2;
    this->marker_follow_cutoff = field_dimensions.width() / 2;
}

std::optional<RobotIntent> Marker::get_task(RobotIntent intent, const WorldState* world_state,
                                            [[maybe_unused]] FieldDimensions field_dimensions) {

    this->y_bound = field_dimensions.length() / 2;
    this->marker_follow_cutoff = field_dimensions.width() / 2;

    rj_geometry::Point targetPoint = world_state->get_robot(false, target_).pose.position();
    rj_geometry::Point ballPoint = world_state->ball.position;
    rj_geometry::Point targetToBall = (ballPoint - targetPoint).normalized(0.55f);
    planning::LinearMotionInstant goal{targetPoint + targetToBall, rj_geometry::Point{0.0, 0.0}};
    intent.motion_command =
        planning::MotionCommand{"path_target", goal, planning::FaceBall{}, true};
    return intent;
}

void Marker::choose_target(const WorldState* ws) {
    // TODO: (James Vogt, github: jvogt23)
    // If we ever use multiple Markers, they should choose different
    // robots to track from each other. Logic for this operation must be
    // added because multiple markers currently mark the same robot.
    for (int i = 0; i < kNumShells; i++) {
        if (std::fabs(ws->get_robot(false, i).pose.position().x()) < marker_follow_cutoff &&
            ws->get_robot(false, i).pose.position().y() < y_bound &&
            (ws->ball.position - ws->get_robot(false, i).pose.position()).mag() > .25) {
            target_ = i;
            return; 
        }
    }
    target_ = -1;
}

bool Marker::target_out_of_bounds(const WorldState* ws) {
    if (target_ == -1) return true;
    if (ws->get_robot(false, target_).pose.position().y() > y_bound) {
        target_ = -1;
        return true;
    }
    return false;
}

int Marker::get_target() { return target_; }
}  // namespace strategy