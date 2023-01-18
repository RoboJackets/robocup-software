#include "motion_command.hpp"

namespace planning {

/* Kevin: there is some built-in operator== for std::variants that (1) checks
 * the type of the variant and (2) if the types match, delegates to any
 * overloaded operator== that compares some specific type of the variant.
 *
 * So MotionCommand and PathTargetFaceOption don't need an operator== explicitly;
 * specifying one for each of the possible types of each works the same.
 */

bool operator==(const LinearMotionInstant& a, const LinearMotionInstant& b) {
    // TODO: this should be in instant.hpp/cpp
    return a.velocity == b.velocity && a.position == b.position;
}

bool operator==(const PathTargetMotionCommand& a, const PathTargetMotionCommand& b) {
    return a.goal == b.goal && a.face_option == b.face_option;
}

bool operator==([[maybe_unused]] const GoalieIdleMotionCommand& a,
                [[maybe_unused]] const GoalieIdleMotionCommand& b) {
    return true;
}

bool operator==([[maybe_unused]] const FaceTarget& a, [[maybe_unused]] const FaceTarget& b) {
    return true;
}
bool operator==(const FaceAngle& a, const FaceAngle& b) { return a.target == b.target; }
bool operator==(const FacePoint& a, const FacePoint& b) {
    double tolerance = 0.1;
    return a.face_point.nearly_equals(b.face_point, tolerance);
}
bool operator==([[maybe_unused]] const FaceBall& a, [[maybe_unused]] const FaceBall& b) {
    return true;
}

bool operator==([[maybe_unused]] const EmptyMotionCommand& a,
                [[maybe_unused]] const EmptyMotionCommand& b) {
    return true;
}

bool operator==(const WorldVelMotionCommand& a, const WorldVelMotionCommand& b) {
    return a.world_vel == b.world_vel;
}
bool operator==(const PivotMotionCommand& a, const PivotMotionCommand& b) {
    return a.pivot_target == b.pivot_target && a.pivot_point == b.pivot_point;
}
bool operator==(const SettleMotionCommand& a, const SettleMotionCommand& b) {
    return a.target == b.target;
}
bool operator==([[maybe_unused]] const CollectMotionCommand& a,
                [[maybe_unused]] const CollectMotionCommand& b) {
    return true;
}
bool operator==(const LineKickMotionCommand& a, const LineKickMotionCommand& b) {
    return a.target == b.target;
}
bool operator==(const InterceptMotionCommand& a, const InterceptMotionCommand& b) {
    return a.target == b.target;
}

}  // namespace planning
