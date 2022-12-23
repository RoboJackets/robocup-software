#include "motion_command.hpp"

namespace planning {

template <class T>
bool are_equivalent(T const& l, T const& r) {
    // (2) which will then delegate to T's overloaded==
    return l == r;
}
bool are_equivalent(MotionCommand const& l, MotionCommand const& r) {
    return std::visit(
        [&](auto const& l, auto const& r) {
            // (1) if types of l/r are the same, they will match to are_equivalent's signature
            return are_equivalent(l, r);
        },
        l, r);
}

bool operator==(const MotionCommand& a, const MotionCommand& b) {
    // if MotionCommand variant is type T, delegate the comparison to T's overloaded ==
    // credit: https://devblogs.microsoft.com/oldnewthing/20190620-00/?p=102604#comment-135008
    return are_equivalent(a, b);
}

// TODO(Kevin): do something similar for AngleOverride (also a variant)

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
