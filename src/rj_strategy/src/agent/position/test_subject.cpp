#include "rj_strategy/agent/position/test_subject.hpp"

#include <rj_geometry/util.hpp>

namespace {
// Stage counters for TestSubject states.
int stage_IDLE = 1;
int stage_YELLOW_STRAIGHT = 1;
int stage_BLUE_STRAIGHT = 1;
int stage_BORDER_FIELD = 1;
}  // namespace

namespace strategy {

TestSubject::TestSubject(int r_id) : Position{r_id, "TestSubject"} {}

TestSubject::TestSubject(Position&& other) : Position{std::move(other)} {
    position_name_ = "TestSubject";
}

std::string TestSubject::get_current_state() { return "TestSubject"; }

std::optional<RobotIntent> TestSubject::derived_get_task(RobotIntent intent) {
    // Update finite state first, then convert it to a command.
    State new_state = next_state();
    current_state_ = new_state;
    return state_to_task(intent);
}

TestSubject::State TestSubject::next_state() {
    // Keep a local copy of the latest test mode selected in UI/config.
    motion_test_type_value_ = motion_test_type();

    switch (motion_test_type_value_) {
        case MotionTestType::NONE: {
            stage_IDLE = 1;
            return IDLE;
        }
        case MotionTestType::YELLOW_STRAIGHT: {
            if (check_is_done()) {
                stage_YELLOW_STRAIGHT++;
            }
            return YELLOW_STRAIGHT;
        }
        case MotionTestType::BLUE_STRAIGHT: {
            if (check_is_done()) {
                stage_BLUE_STRAIGHT++;
            }
            return BLUE_STRAIGHT;
        }
        case MotionTestType::BORDER_FIELD: {
            if (check_is_done()) {
                stage_BORDER_FIELD++;
            }
            return BORDER_FIELD;
        }
        case MotionTestType::DEFAULT_FIELD: {
            return IDLE;
        }
        default: {
            return IDLE;
        }
    }
}

std::optional<RobotIntent> TestSubject::state_to_task(RobotIntent intent) {
    rj_geometry::Point target{0.0, 0.0};
    rj_geometry::Point target_vel{0.0, 0.0};
    planning::PathTargetFaceOption face_option{planning::FaceAngle{0.0}};

    switch (current_state_) {
        case YELLOW_STRAIGHT: {
            switch (stage_YELLOW_STRAIGHT) {
                case 1:
                    target = rj_geometry::Point{2.0, 1.6};
                    face_option = planning::FaceAngle{0.0};
                    break;
                case 2:
                    target = rj_geometry::Point{-2.0, 1.6};
                    face_option = planning::FaceAngle{M_PI};
                    break;
                default: {
                    stage_YELLOW_STRAIGHT = 1;
                    target = rj_geometry::Point{2.0, 1.6};
                    face_option = planning::FaceAngle{0.0};
                    break;
                }
            }
            planning::LinearMotionInstant goal{target, target_vel};
            intent.motion_command = planning::MotionCommand{"path_target_test", goal, face_option};
            return intent;
        }
        case BLUE_STRAIGHT: {
            switch (stage_BLUE_STRAIGHT) {
                case 1:
                    target = rj_geometry::Point{2.0, 7.6};
                    face_option = planning::FaceAngle{0.0};
                    break;
                case 2:
                    target = rj_geometry::Point{-2.0, 7.6};
                    face_option = planning::FaceAngle{M_PI};
                    break;
                default: {
                    stage_BLUE_STRAIGHT = 1;
                    target = rj_geometry::Point{2.0, 7.6};
                    face_option = planning::FaceAngle{0.0};
                    break;
                }
            }
            planning::LinearMotionInstant goal{target, target_vel};
            intent.motion_command = planning::MotionCommand{"path_target_test", goal, face_option};
            return intent;
        }
        case BORDER_FIELD: {
            switch (stage_BORDER_FIELD) {
                case 1:
                    target = rj_geometry::Point{-2.6, 0.5};
                    face_option = planning::FaceAngle{-M_PI/2};
                    break;
                case 2:
                    target = rj_geometry::Point{2.6, 0.5};
                    face_option = planning::FaceAngle{0.0};
                    break;
                case 3:
                    target = rj_geometry::Point{2.6, 8.5};
                    face_option = planning::FaceAngle{M_PI/2};
                    break;
                case 4:
                    target = rj_geometry::Point{-2.6, 8.5};
                    face_option = planning::FaceAngle{M_PI};
                    break;
                default: {
                    stage_BORDER_FIELD = 1;
                    target = rj_geometry::Point{-2.6, 0.5};
                    face_option = planning::FaceAngle{-M_PI/2};
                    break;
                }
            }
            planning::LinearMotionInstant goal{target, target_vel};
            intent.motion_command = planning::MotionCommand{"path_target_test", goal, face_option};
            return intent;
        }
        default: {
            intent.motion_command = planning::MotionCommand{};
            return intent;
        }
    }
    return intent;
}

}  // namespace strategy

