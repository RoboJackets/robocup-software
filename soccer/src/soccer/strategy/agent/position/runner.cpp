#include "runner.hpp"
#include "planning/instant.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id) {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    WorldState* world_state = this->world_state();
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    switch (latest_state_) {
        case IDLING:
            initial_position_ = robot_position;
            return RUNFIRSTSIDE;
        case RUNFIRSTSIDE:
            if ((robot_position.y() - initial_position_.y()) > kSideLength) {
                initial_position_ = robot_position;
                return RUNSECONDSIDE;
            }
        case RUNSECONDSIDE:
            if ((initial_position_.x() - robot_position.x()) > kSideLength) {
                initial_position_ = robot_position;
                return RUNTHIRDSIDE;
            }
        case RUNTHIRDSIDE:
            if ((initial_position_.y() - robot_position.y()) > kSideLength) {
                initial_position_ = robot_position;
                return RUNFOURTHSIDE;
            }
        case RUNFOURTHSIDE:
            if ((robot_position.x() - initial_position_.x() > kSideLength)) {
                initial_position_ = robot_position;
                return IDLING;
            }
        default:
            return latest_state_;
    }
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    planning::LinearMotionInstant target;
    switch (latest_state_) {
        case IDLING:
            return intent;
        case RUNFIRSTSIDE:
            target = planning::LinearMotionInstant(
                    rj_geometry::Point(initial_position_.x(), initial_position_.y() + kSideLength));
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceTarget{}, true};
            return intent;
        case RUNSECONDSIDE:
            target = planning::LinearMotionInstant(
                    rj_geometry::Point(initial_position_.x() - kSideLength, initial_position_.y()));
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceTarget{}, true};
            return intent;
        case RUNTHIRDSIDE:
            target = planning::LinearMotionInstant(
                    rj_geometry::Point(initial_position_.x(), initial_position_.y() - kSideLength));
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceTarget{}, true};
            return intent;
        case RUNFOURTHSIDE:
            target = planning::LinearMotionInstant(
                    rj_geometry::Point(initial_position_.x() + kSideLength, initial_position_.y()));
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceTarget{}, true};
            return intent;
        default:
            return intent;
    }
}

}  // namespace strategy
