#include "trajectory.hpp"

#include <stdexcept>

#include <spdlog/spdlog.h>

#include <rj_geometry/pose.hpp>

#include "instant.hpp"

namespace planning {

using rj_geometry::Pose;
using rj_geometry::Twist;

Trajectory::Trajectory(Trajectory a, const Trajectory& b) {
    if (a.empty() || b.empty()) {
        throw std::invalid_argument("Cannot splice empty trajectories");
    }

    RobotInstant a_end = a.last();
    RobotInstant b_begin = b.first();

    using rj_geometry::Point;
    if (!a_end.position().near_point(b_begin.position(), 1e-6) ||
        !a_end.linear_velocity().near_point(b_begin.linear_velocity(), 1e-6) ||
        a_end.stamp != b_begin.stamp) {
        SPDLOG_ERROR("points near? {}, vels near? {}, timestamps match? {}",
                     !a_end.position().near_point(b_begin.position(), 1e-6),
                     !a_end.linear_velocity().near_point(b_begin.linear_velocity(), 1e-6),
                     a_end.stamp != b_begin.stamp);
        throw std::invalid_argument(
            "Cannot splice trajectories a and b, where a.last() != b.first()");
    }

    instants_ = std::move(a.instants_);
    instants_.reserve(instants_.size() + b.instants_.size() - 1);
    instants_.insert(instants_.end(), b.instants_.begin() + 1, b.instants_.end());

    has_angle_profile_ = false;
    creation_stamp_ = std::nullopt;
}

void Trajectory::append_instant(RobotInstant instant) {
    if (!empty() && instant.stamp <= end_time()) {
        throw std::invalid_argument(
            "Cannot append instant at or before the "
            "last instant in the trajectory");
    }

    instants_.push_back(instant);

    // This is not profiled correctly, so we should mark the entire
    // trajectory's angle profile as invalid.
    has_angle_profile_ = false;
    creation_stamp_ = std::nullopt;
}

bool Trajectory::check_time(RJ::Time time) const {
    return time >= begin_time() && time <= end_time();
}

bool Trajectory::check_seconds(RJ::Seconds seconds) const {
    return seconds >= 0s && seconds <= duration();
}

void Trajectory::scale_duration(RJ::Seconds final_duration) {
    // Note: even though this modifies the trajectory, it does not break
    // angle planning and so we do not need to mark the angle profile as
    // invalid.
    scale_duration(final_duration, begin_time());
}

void Trajectory::scale_duration(RJ::Seconds final_duration, RJ::Time fixed_point) {
    // Note: even though this modifies the trajectory, it does not break
    // angle planning and so we do not need to mark the angle profile as
    // invalid.
    double multiplier = final_duration / duration();

    for (RobotInstant& instant : instants_) {
        instant.velocity /= multiplier;
        instant.stamp = fixed_point + RJ::Seconds(instant.stamp - fixed_point) * multiplier;
    }

    creation_stamp_ = std::nullopt;
}

std::optional<RobotInstant> Trajectory::evaluate(RJ::Seconds seconds) const {
    return evaluate(begin_time() + seconds);
}

std::optional<RobotInstant> Trajectory::evaluate(RJ::Time time) const {
    if (instants_.empty() || !check_time(time)) {
        return std::nullopt;
    }

    Cursor cursor(*this);
    cursor.seek(time);
    return cursor.value();
}

RobotInstant Trajectory::interpolated_instant(const RobotInstant& prev_entry,
                                              const RobotInstant& next_entry, RJ::Time time) {
    if (time < prev_entry.stamp || time > next_entry.stamp) {
        throw std::invalid_argument("Interpolant time is not between prev_ and next_ timestamps");
    }

    if (time == prev_entry.stamp) {
        return prev_entry;
    }

    if (time == next_entry.stamp) {
        return next_entry;
    }

    RJ::Seconds dt = next_entry.stamp - prev_entry.stamp;
    RJ::Seconds elapsed = time - prev_entry.stamp;

    // s in [0, 1] is the interpolation factor.
    double s = elapsed / dt;
    if (s < 0 || s > 1) {
        throw std::runtime_error("Interpolant `s` is out of bounds!");
    }

    Pose pose_0 = prev_entry.pose;
    Pose pose_1 = next_entry.pose;
    Twist tangent_0 = prev_entry.velocity * RJ::num_seconds(dt);
    Twist tangent_1 = next_entry.velocity * RJ::num_seconds(dt);

    // Cubic interpolation.
    // We've rescaled the problem to exist in the range [0, 1] instead of
    // [t0, t1] by adjusting the tangent vectors, so now we can interpolate
    // using a Hermite spline. The coefficients for `interpolated_pose` can be
    // found at https://en.wikipedia.org/wiki/Cubic_Hermite_spline. The
    // coefficients for `interpolated_twist` are chosen to be the derivative of
    // `interpolated_pose` with respect to s, and then it is rescaled to match
    // the time derivative
    Pose interpolated_pose = Pose(Eigen::Vector3d(pose_0) * (2 * s * s * s - 3 * s * s + 1) +
                                  Eigen::Vector3d(tangent_0) * (s * s * s - 2 * s * s + s) +
                                  Eigen::Vector3d(pose_1) * (-2 * s * s * s + 3 * s * s) +
                                  Eigen::Vector3d(tangent_1) * (s * s * s - s * s));

    Twist interpolated_twist = Twist(Eigen::Vector3d(pose_0) * (6 * s * s - 6 * s) +
                                     Eigen::Vector3d(tangent_0) * (3 * s * s - 4 * s + 1) +
                                     Eigen::Vector3d(pose_1) * (-6 * s * s + 6 * s) +
                                     Eigen::Vector3d(tangent_1) * (3 * s * s - 2 * s)) /
                               RJ::num_seconds(dt);

    // Create a new RobotInstant with the correct values.
    return RobotInstant{interpolated_pose, interpolated_twist, time};
}

Trajectory Trajectory::sub_trajectory(RJ::Time clip_start_time, RJ::Time clip_end_time) const {
    if (clip_start_time > clip_end_time) {
        throw std::invalid_argument("End time must not come before start time");
    }

    Cursor cursor(*this, clip_start_time);

    if (!cursor.has_value()) {
        throw std::invalid_argument("Sub-trajectory start time is outside of trajectory");
    }

    RJ::Time actual_end = std::min(clip_end_time, end_time());

    // If the start and end times are identical, we are trying to grab an
    // infinitesimal trajectory. This is weird but technically not wrong.
    if (clip_start_time == actual_end) {
        return Trajectory({cursor.value()});
    }

    RobotInstantSequence result_instants;
    // TODO(#1502): Reserve space in result_instants.
    while (cursor.has_value() && cursor.value().stamp < actual_end) {
        result_instants.push_back(cursor.value());
        cursor.next_knot();
    }

    // The above calculation will always miss the last instant.
    result_instants.push_back(
        Trajectory::interpolated_instant(result_instants.back(), cursor.value(), actual_end));

    return Trajectory{std::move(result_instants)};
}

Trajectory::Cursor Trajectory::cursor(RJ::Time start_time) const {
    return Cursor{*this, start_time};
}

Trajectory::Cursor Trajectory::cursor_begin() const { return Cursor{*this, instants_begin()}; }

void Trajectory::draw(DebugDrawer* drawer,
                      std::optional<rj_geometry::Point> alt_text_position) const {
    if (instants_.size() > 1) {
        Packet::DebugRobotPath* dbg_path = drawer->add_debug_path();
        dbg_path->set_layer(drawer->find_debug_layer("Motion"));

        for (const RobotInstant& instant : instants_) {
            Packet::DebugRobotPath::DebugRobotPathPoint* pt = dbg_path->add_points();
            *pt->mutable_pos() = instant.pose.position();
            *pt->mutable_vel() = instant.velocity.linear();
        }
    }

    if (debug_text_) {
        rj_geometry::Point text_pos;

        // Only use the backup position if there's no trajectory.
        if (!empty()) {
            text_pos = first().pose.position() + rj_geometry::Point(0.1, 0);
        } else if (alt_text_position.has_value()) {
            text_pos = alt_text_position.value();
        } else {
            return;
        }

        drawer->draw_text(QString::fromStdString(debug_text_.value()), text_pos,
                          QColor(100, 100, 255, 100), "PlanningDebugText");
    }
}

bool Trajectory::nearly_equal(const Trajectory& a, const Trajectory& b, double tolerance) {
    auto a_it = a.instants_begin();
    auto b_it = b.instants_begin();
    for (; a_it != a.instants_end() && b_it != b.instants_end(); ++a_it, ++b_it) {
        if (!RobotInstant::nearly_equals(*a_it, *b_it, tolerance)) {
            return false;
        }
    }

    // If there is more time to go in either trajectory, they are different.
    // Otherwise, they are identical.
    return a_it == a.instants_end() && b_it == b.instants_end();
}
void Trajectory::hold_for(RJ::Seconds duration) {
    RobotInstant instant = last();
    if (duration <= RJ::Seconds(0)) {
        throw std::invalid_argument("Hold duration must be positive");
    }

    if (!Twist::nearly_equals(instant.velocity, Twist::zero())) {
        throw std::runtime_error("Cannot hold nonzero velocity");
    }

    instant.velocity = Twist::zero();
    instant.stamp = instant.stamp + duration;

    // Does not invalidate angle planning.
    instants_.push_back(instant);
}

Trajectory::Cursor::Cursor(const Trajectory& trajectory,
                           RobotInstantSequence::const_iterator iterator)
    : trajectory_{trajectory}, iterator_{iterator}, time_{iterator->stamp} {}

Trajectory::Cursor::Cursor(const Trajectory& trajectory)
    : Cursor{trajectory, trajectory.instants_.begin()} {}

Trajectory::Cursor::Cursor(const Trajectory& trajectory, RJ::Time start_time) : Cursor{trajectory} {
    seek(start_time);
}

RobotInstant Trajectory::Cursor::value() const {
    RobotInstant instant_0 = *iterator_;
    if (instant_0.stamp == time_) {
        return instant_0;
    }
    RobotInstant instant_1 = *(iterator_ + 1);

    return Trajectory::interpolated_instant(instant_0, instant_1, time_);
}

void Trajectory::Cursor::seek(RJ::Time time) {
    // If we are seeking past the end (or before the beginning) it's not an
    // error, we just don't have a valid value anymore. We do this to be
    // consistent with the behavior of advance().
    if (!trajectory_.check_time(time)) {
        time_ = time;
        iterator_ = trajectory_.instants_end();
        return;
    }

    // Note that this comparison is less than or equal to.
    auto compare_times = [](const RobotInstant& a, const RobotInstant& b) -> bool {
        return a.stamp <= b.stamp;
    };

    RobotInstant dummy_instant;
    dummy_instant.stamp = time;

    // std::lower_bound finds the first iterator such that the above comparison
    // _fails_ - that is, the first instant with time < instant.stamp.
    // We actually want the _last_ instant with instant.stamp <= time, so we
    // can use the call to lower_bound and then take a step back.
    iterator_ = std::lower_bound(trajectory_.instants_.begin(), trajectory_.instants_.end(),
                                 dummy_instant, compare_times);

    if (iterator_ == trajectory_.instants_.begin()) {
        throw std::runtime_error(
            "Cannot seek before beginning of trajectory. This should be "
            "unreachable.");
    }
    iterator_--;

    time_ = time;
}

void Trajectory::Cursor::advance(RJ::Seconds seconds) {
    time_ = RJ::Time(time_ + seconds);

    if (!trajectory_.check_time(time_)) {
        time_ = RJ::Time::max();
        return;
    }

    if (seconds < RJ::Seconds(0)) {
        throw std::invalid_argument("Argument `seconds` must not be negative.");
    }

    // If we're moving forwards, we want to step for as long as our current
    // knot point is _before or at_ the target time. This will leave us with
    // a step one past the proper start instant, so we then step back.
    while (iterator_ != trajectory_.instants_.end() && iterator_->stamp <= time_) {
        iterator_++;
    }

    if (iterator_ == trajectory_.instants_.begin()) {
        throw std::runtime_error(
            "Cannot seek before beginning of trajectory. This should be "
            "unreachable.");
    }
    iterator_--;
}

void Trajectory::Cursor::next_knot() {
    if (iterator_ != trajectory_.instants_end()) {
        iterator_++;
    }

    if (iterator_ == trajectory_.instants_end()) {
        time_ = RJ::Time::max();
    } else {
        time_ = iterator_->stamp;
    }
}

}  // namespace planning
