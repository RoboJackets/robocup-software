#include "pivot_kick_path_planner.hpp"

PivotKickPathPlanner::PivotKickPathPlanner() { current_state_ = CAPTURE; }

PivotKickPathPlanner::State PivotKickPathPlanner::update_state() {
    if (subskill_is_done_) {
        if (current_state_ == CAPTURE) {
            current_state_ = PIVOT;
        } else if (current_state_ == PIVOT) {
            current_state_ = KICK;
        } else if (current_state_ == KICK) {
            current_state_ = DONE;
        } else if (current_state_ == DONE) {
            current_state_ = DONE;
        }
    }
}

Trajectory PivotKickPathPlanner::plan(const PlanRequest& plan_request) {
    if (current_state_ == CAPTURE) {
    } else if (current_state_ == PIVOT) {
    } else if (current_state_ == KICK) {
    } else if (current_state_ == DONE) {
    }
}

void PivotKickPathPlanner::reset() { current_state_ = CAPTURE; }

bool PivotKickPathPlanner::is_done() const { return current_state_ == DONE; }
