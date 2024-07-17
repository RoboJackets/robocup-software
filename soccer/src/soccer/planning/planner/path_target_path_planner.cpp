#include "planning/planner/path_target_path_planner.hpp"

#include <utility>

#include <spdlog/spdlog.h>

#include "planning/instant.hpp"
#include "planning/planner/plan_request.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "planning/trajectory.hpp"

using namespace rj_geometry;

namespace planning {

Trajectory PathTargetPathPlanner::plan(const PlanRequest& request) {
    // Collect obstacles
    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    Trajectory ball_trajectory;
    const MotionCommand& command = request.motion_command;
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, !command.ignore_ball,
                   &ball_trajectory);

    // If we start inside of an obstacle, give up and let another planner take
    // care of it.
    if (static_obstacles.hit(request.start.position())) {
        reset();
        return Trajectory();
    }

    LinearMotionInstant target_instant = command.target;
    Point goal_point = target_instant.position;

    // Cache the start and goal instants for is_done()
    cached_target_instant_ = target_instant;
    cached_start_instant_ = request.start.linear_motion();

    AngleFunction angle_function = get_angle_function(request);

    // Call into the sub-object to actually execute the plan.
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{request.start, target_instant, static_obstacles, dynamic_obstacles,
                              request.constraints, angle_function, RJ::Seconds(3.0)},
        std::move(previous_));

    previous_ = trajectory;
    return trajectory;
}

bool PathTargetPathPlanner::is_done() const {
    if (!cached_start_instant_.has_value() || !cached_target_instant_.has_value()) {
        return false;
    }

    // TODO(Kevin): also, should enforce the desired angle
    // right now there is a convoluted chain
    // PathTargetPathPlanner->Replanner->plan_angles which plans angles depending
    // on AngleFunction (either desired face point or desired face angle).
    // nowhere in the chain is there a check if PathTargetPathPlanner actually is
    // getting to the desired angle.
    //
    // may be related to issue #1506?
    double position_tolerance = 0.1;
    double velocity_tolerance = 0.1;
    return LinearMotionInstant::nearly_equals(cached_start_instant_.value(),
                                              cached_target_instant_.value(), position_tolerance,
                                              velocity_tolerance);
    // TODO(Kevin): in theory this should work as LinearMotionInstant has
    // tolerance built into its == overload, but in practice it doesn't
    /* return cached_start_instant_ == cached_target_instant_; */
}

AngleFunction PathTargetPathPlanner::get_angle_function(const PlanRequest& request) {
    const auto& face_option = request.motion_command.face_option;

    if (std::holds_alternative<FacePoint>(face_option)) {
        return AngleFns::face_point(std::get<FacePoint>(face_option).face_point);
    }

    if (std::holds_alternative<FaceBall>(face_option)) {
        auto ball_pos = request.world_state->ball.position;
        return AngleFns::face_point(ball_pos);
    }

    if (std::holds_alternative<FaceAngle>(face_option)) {
        return AngleFns::face_angle(std::get<FaceAngle>(face_option).target);
    }

    // default to facing tangent to path
    // (rj_convert in motion_command.hpp also follows this default)
    return AngleFns::tangent;
}

}  // namespace planning
