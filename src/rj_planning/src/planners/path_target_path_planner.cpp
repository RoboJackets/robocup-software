#include "rj_planning/planners/path_target_path_planner.hpp"

#include <rj_common/utils.hpp>

using namespace rj_geometry;

namespace planning {

Trajectory PathTargetPathPlanner::plan(const PlanRequest& request) {
    // Collect obstacles
    ObstacleSet obstacles;
    const MotionCommand& command = request.motion_command;
    fill_obstacles(request, obstacles, !command.ignore_ball);

    // If we start inside of an obstacle, give up and let another planner take
    // care of it.
    if (obstacles.hit(request.start.position())) {
        reset();
        return Trajectory();
    }

    LinearMotionInstant target_instant = command.target;
    Point goal_point = target_instant.position;

    // Cache the start and goal instants for is_done()
    cached_target_instant_ = target_instant;
    cached_start_instant_ = request.start.linear_motion();
    cached_start_heading_ = request.start.heading();

    AngleFunction angle_function = get_angle_function(request);

    LinearMotionInstant target_for_angle{target_instant.position, rj_geometry::Point{0, 0}};
    cached_target_angle_ = angle_function(target_for_angle, cached_start_heading_.value(), nullptr);

    // Call into the sub-object to actually execute the plan.
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{request.start, target_instant, obstacles, request.field_dimensions,
                              request.constraints, angle_function, request.shell_id,
                              RJ::Seconds(3.0)},
        std::move(previous_));

    previous_ = trajectory;
    return trajectory;
}

bool PathTargetPathPlanner::is_done() const {
    if (!cached_start_instant_.has_value() || !cached_target_instant_.has_value()) {
        return false;
    }

    double position_tolerance = 1e-2;
    double velocity_tolerance = 1e-2;
    if (!LinearMotionInstant::nearly_equals(cached_start_instant_.value(),
                                            cached_target_instant_.value(), position_tolerance,
                                            velocity_tolerance)) {
        return false;
    }

    if (cached_start_heading_.has_value() && cached_target_angle_.has_value()) {
        constexpr double angle_tolerance = 0.1;
        if (std::abs(fix_angle_radians(cached_start_heading_.value() -
                                       cached_target_angle_.value())) > angle_tolerance) {
            return false;
        }
    }

    return true;
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

    if (std::holds_alternative<FaceTarget>(face_option)) {
        return AngleFns::face_point(request.motion_command.target.position);
    }

    // default to facing tangent to path
    // (rj_convert in motion_command.hpp also follows this default)
    return AngleFns::tangent;
}

}  // namespace planning
