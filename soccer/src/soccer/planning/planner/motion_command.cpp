#include "motion_command.hpp"

namespace planning {

bool operator==(const LinearMotionInstant& a, const LinearMotionInstant& b) {
    return a.velocity == b.velocity && a.position == b.position;
}

bool operator==(const PathTargetCommand& a, const PathTargetCommand& b) {
    SPDLOG_ERROR("ptc == reached");
    return a.goal == b.goal && a.angle_override == b.angle_override;
}

bool operator==([[maybe_unused]] const GoalieIdleCommand& a,
                [[maybe_unused]] const GoalieIdleCommand& b) {
    return true;
}

bool operator==([[maybe_unused]] const TargetFaceTangent& a,
                [[maybe_unused]] const TargetFaceTangent& b) {
    return true;
}
bool operator==(const TargetFaceAngle& a, const TargetFaceAngle& b) { return a.target == b.target; }
bool operator==(const TargetFacePoint& a, const TargetFacePoint& b) {
    return a.face_point == b.face_point;
}

}  // namespace planning
