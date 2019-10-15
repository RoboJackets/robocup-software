#include "Trajectory.hpp"

namespace Planning {

using Geometry2d::Pose;
using Geometry2d::Twist;

void Trajectory::InsertInstant(RobotInstant instant) {
    instants_.insert(std::upper_bound(
                    instants_.begin(),
                    instants_.end(),
                    instant,
                    [](RobotInstant a, RobotInstant b) {
                        return a.stamp < b.stamp;
                    }), instant);
}

void Trajectory::AppendInstant(RobotInstant instant) {
    assert(empty() || instant.stamp > end_time());

    instants_.push_back(instant);
}

bool Trajectory::CheckTime(RJ::Time time) const {
    return time >= begin_time() && time <= end_time();
}

bool Trajectory::CheckSeconds(RJ::Seconds seconds) const {
    return seconds >= 0s && seconds <= duration();
}

void Trajectory::ScaleDuration(RJ::Seconds final_duration) {
    ScaleDuration(final_duration, begin_time());
}

void Trajectory::ScaleDuration(RJ::Seconds final_duration, RJ::Time fixed_point) {
    double multiplier = final_duration / duration();

    for (RobotInstant &instant : instants_) {
        instant.velocity /= multiplier;
        instant.stamp = fixed_point + RJ::Seconds(instant.stamp - fixed_point) * multiplier;
    }
}

std::optional<RobotInstant> Trajectory::EvaluateSeconds(RJ::Seconds seconds) const {
    if (instants_.empty()) {
        return std::nullopt;
    }

    return EvaluateTime(begin_time() + seconds);
}

std::optional<RobotInstant> Trajectory::EvaluateTime(RJ::Time time) const {
    if (instants_.empty()) {
        return std::nullopt;
    }

    if (time == begin_time()) {
        return instants_.front();
    }

    if (time == end_time()) {
        return instants_.back();
    }

    if (time < begin_time() || time > end_time()) {
        return std::nullopt;
    }

    // Find the waypoints on either side of the query time such that
    // prev_it->time < t <= next_it->time
    std::vector<RobotInstant>::const_iterator prev_it = instants_.begin();
    std::vector<RobotInstant>::const_iterator next_it = instants_.begin();
    while (next_it != instants_.end()) {
        if (next_it->stamp >= time) {
            break;
        }

        prev_it = next_it;
        next_it++;
    }

    // It shouldn't be possible for this to occur based on the checks at the
    // beginning of this method.
    assert(prev_it != instants_.end());
    assert(next_it != instants_.end());

    RobotInstant prev_entry = *prev_it;
    RobotInstant next_entry = *next_it;

    RJ::Seconds dt = next_entry.stamp - prev_entry.stamp;
    if (dt == RJ::Seconds(0)) {
        return next_entry;
    }
    RJ::Seconds elapsed = time - prev_entry.stamp;

    // s in [0, 1] is the interpolation factor.
    double s = elapsed / dt;

    Pose pose_0 = prev_entry.pose;
    Pose pose_1 = next_entry.pose;
    Twist tangent_0 = prev_entry.velocity * RJ::numSeconds(dt);
    Twist tangent_1 = next_entry.velocity * RJ::numSeconds(dt);

    // Cubic interpolation.
    // We've rescaled the problem to exist in the range [0, 1] instead of
    // [t0, t1] by adjusting the tangent vectors, so now we can interpolate
    // using a Hermite spline. The coefficients for `interpolated_pose` can be
    // found at https://en.wikipedia.org/wiki/Cubic_Hermite_spline. The
    // coefficients for `interpolated_twist` are chosen to be the derivative of
    // `interpolated_pose` with respect to s, and then it is rescaled to match
    // the time derivative
    Pose interpolated_pose =
            Pose(Eigen::Vector3d(pose_0) * (2 * s * s * s - 3 * s * s + 1) +
                 Eigen::Vector3d(tangent_0) * (s * s * s - 2 * s * s + s) +
                 Eigen::Vector3d(pose_1) * (-2 * s * s * s + 3 * s * s) +
                 Eigen::Vector3d(tangent_1) * (s * s * s - s * s));

    Twist interpolated_twist =
            Twist(Eigen::Vector3d(pose_0) * (6 * s * s - 6 * s) +
                  Eigen::Vector3d(tangent_0) * (3 * s * s - 4 * s + 1) +
                  Eigen::Vector3d(pose_1) * (-6 * s * s + 6 * s) +
                  Eigen::Vector3d(tangent_1) * (3 * s * s - 2 * s)) /
            RJ::numSeconds(dt);

    // Create a new RobotInstant with the correct values.
    return RobotInstant{interpolated_pose, interpolated_twist, time};
}

} // namespace Planning
