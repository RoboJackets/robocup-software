#include "rj_control/controllers/mark_robot_controller.hpp"

namespace control {

ControlCommand MarkRobotController::update(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return {{0.0, 0.0, 0.0}};
    }

    const rj_geometry::Pose& current_pose = world_state.get_robot(true, robot_id_).pose;
    const rj_geometry::Pose& their_pose = world_state.get_robot(false, action.mark_robot()->their_robot_id()).pose;

    return go_to_pose_.update(
        robot_id_,
        world_state,
        calculate_target_pose(current_pose, their_pose, field_dimensions, action.mark_robot()->distance()),
        x_controller_,
        y_controller_,
        w_controller_
    );
}

bool MarkRobotController::complete(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return true;
    }

    return go_to_pose_.complete(
        robot_id_,
        world_state,
        calculate_target_pose(
            world_state.get_robot(true, robot_id_).pose,
            world_state.get_robot(false, action.mark_robot()->their_robot_id()).pose,
            field_dimensions,
            action.mark_robot()->distance()
        ),
        std::make_pair(0.1, M_PI / 4)
    );
}

rj_geometry::Pose MarkRobotController::calculate_target_pose(
    const rj_geometry::Pose& our_pose,
    const rj_geometry::Pose& their_pose,
    const FieldDimensions& field_dimensions,
    double distance
) {
    return {
        their_pose.position() + (field_dimensions.our_goal_loc() - their_pose.position()) * distance,
        (their_pose.position() - our_pose.position()).angle()
    };
}

} // namespace control