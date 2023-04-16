#include "pivot_kick_path_planner.hpp"

#include <spdlog/spdlog.h>

namespace planning {

PivotKickPathPlanner::State PivotKickPathPlanner::update_state() {
    if (current_state_ == State::SETTLE) {
        if (settle_path_planner_.is_done()) {
            current_state_ = State::COLLECT;
        }
    } else if (current_state_ == State::COLLECT) {
        if (collect_path_planner_.is_done()) {
            current_state_ = State::PIVOT;
        }
    } else if (current_state_ == State::PIVOT) {
        if (pivot_path_planner_.is_done()) {
            // TODO: ManipulatorSetpoint only exists in RobotIntent, so we
            // can't set it via planning, which is stupid
            // for now I hack around it (see Offense.cpp)

            /* current_state_ = State::KICK; */
            current_state_ = State::DONE;
        }
        /* } else if (current_state_ == State::KICK) { */
        /*     current_state_ = State::DONE; */
    } else if (current_state_ == State::DONE) {
        current_state_ = State::DONE;
    }
}

Trajectory PivotKickPathPlanner::plan(const PlanRequest& plan_request) {
    // first, update the current_state_
    update_state();

    // now plan based on current_state_
    SPDLOG_INFO("{}: current_state_ {}", plan_request.shell_id, current_state_);
    if (current_state_ == State::SETTLE) {
        Trajectory plan = settle_path_planner_.plan(plan_request);
        return plan;
    } else if (current_state_ == State::COLLECT) {
        Trajectory plan = collect_path_planner_.plan(plan_request);
        return plan;
    } else if (current_state_ == State::PIVOT) {
        Trajectory plan = pivot_path_planner_.plan(plan_request);
        return plan;
        /* } else if (current_state_ == State::KICK) { */
    } else if (current_state_ == State::DONE) {
        // do nothing
    }
}

void PivotKickPathPlanner::reset() {
    current_state_ = State::SETTLE;
    settle_path_planner_.reset();
    collect_path_planner_.reset();
    pivot_path_planner_.reset();
}

bool PivotKickPathPlanner::is_done() const { return current_state_ == State::DONE; }
}  // namespace planning
