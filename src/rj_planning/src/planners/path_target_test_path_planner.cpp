#include "rj_planning/planners/path_target_test_path_planner.hpp"

using namespace rj_geometry;

namespace planning {

Trajectory PathTargetTestPathPlanner::plan(const PlanRequest& request) {
    const MotionCommand& command = request.motion_command;

    LinearMotionInstant target_instant = command.target;

    // Cache start/goal instants for is_done().
    cached_target_instant_ = target_instant;
    cached_start_instant_ = request.start.linear_motion();

    AngleFunction angle_function = get_angle_function(request);

    // Keep behavior identical to path_target except no obstacles are inserted.
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{request.start,
                              target_instant,
                              ObstacleSet{},
                              request.field_dimensions,
                              request.constraints,
                              angle_function,
                              request.shell_id,
                              RJ::Seconds(3.0)},
        std::move(previous_));

    previous_ = trajectory;
    return trajectory;
}

bool PathTargetTestPathPlanner::is_done() const {
    if (!cached_start_instant_.has_value() || !cached_target_instant_.has_value()) {
        return false;
    }

    double position_tolerance = 1e-1;
    double velocity_tolerance = 1e-1;
    return LinearMotionInstant::nearly_equals(cached_start_instant_.value(),
                                              cached_target_instant_.value(), position_tolerance,
                                              velocity_tolerance);
}

AngleFunction PathTargetTestPathPlanner::get_angle_function(const PlanRequest& request) {
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

    // Default to facing tangent to path.
    return AngleFns::tangent;
}

}  // namespace planning
