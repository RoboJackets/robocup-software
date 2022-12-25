#include "motion_command.hpp"

namespace planning {

/* Kevin: there is some built-in operator== for std::variants that (1) checks
 * the type of the variant and (2) if the types match, delegates to any
 * overloaded operator== that compares some specific type of the variant.
 *
 * So MotionCommand and AngleOverride don't need an operator== explicitly;
 * specifying one for each of the possible types of each works the same.
 */

bool operator==(const LinearMotionInstant& a, const LinearMotionInstant& b) {
    return a.velocity == b.velocity && a.position == b.position;
}

bool operator==(const PathTargetCommand& a, const PathTargetCommand& b) {
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

bool operator==([[maybe_unused]] const EmptyCommand& a, [[maybe_unused]] const EmptyCommand& b) {
    return true;
}

bool operator==(const WorldVelCommand& a, const WorldVelCommand& b) {
    return a.world_vel == b.world_vel;
}
bool operator==(const PivotCommand& a, const PivotCommand& b) {
    return a.pivot_target == b.pivot_target && a.pivot_point == b.pivot_point;
}
bool operator==(const SettleCommand& a, const SettleCommand& b) { return a.target == b.target; }
bool operator==([[maybe_unused]] const CollectCommand& a,
                [[maybe_unused]] const CollectCommand& b) {
    return true;
}
bool operator==(const LineKickCommand& a, const LineKickCommand& b) { return a.target == b.target; }
bool operator==(const InterceptCommand& a, const InterceptCommand& b) {
    return a.target == b.target;
}

}  // namespace planning
