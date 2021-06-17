#include "motion_command.hpp"

namespace planning {

bool operator==(const LinearMotionInstant& a, const LinearMotionInstant& b) {
    // TODO(Kevin): this should be in instant.hpp/cpp
    return a.velocity == b.velocity && a.position == b.position;
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

/*
 * A MotionCommand == another MotionCommand if all the fields are == (with
 * fuzzy floating-point comparison).
 */
bool operator==(const MotionCommand& a, const MotionCommand& b) {
    return a.name == b.name && a.target == b.target && a.face_option == b.face_option &&
           a.ignore_ball == b.ignore_ball && a.pivot_point == b.pivot_point;
}

}  // namespace planning
