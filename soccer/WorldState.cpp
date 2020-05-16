#include "WorldState.hpp"

BallState BallState::predict_at(RJ::Time time) const {
    // If the estimate isn't valid, just return an invalid ball.
    if (!visible) {
        return BallState();
    }

    if (time < timestamp) {
        throw std::runtime_error("Estimated Time can't be before observation time.");
    }

    auto dt = RJ::Seconds(time - timestamp);

    const auto s0 = velocity.mag();

    // If the ball is stopped just return it.
    if (s0 == 0) {
        return *this;
    }

    double speed = 0;
    double distance = 0;

    double max_time = s0 / kBallDecayConstant;
    if (dt.count() >= max_time) {
        speed = 0;
        distance = s0 * max_time - pow(max_time, 2) / 2.0 * kBallDecayConstant;
    } else {
        speed = s0 - (dt.count() * kBallDecayConstant);
        distance = s0 * dt.count() - pow(dt.count(), 2) / 2.0 * kBallDecayConstant;
    }

    return BallState(position + velocity.normalized(distance),
                     velocity.normalized(speed),
                     time);
}

BallState BallState::predict_in(RJ::Seconds seconds) const {
    return predict_at(timestamp + seconds);
}

RJ::Time BallState::query_time_at(Geometry2d::Point near_to,
                                  Geometry2d::Point* out) const {
    // If the ball isn't moving we're as close as we're ever going to get.
    if (velocity.mag() == 0) {
        if (out != nullptr) {
            *out = position;
        }
        return timestamp;
    }

    // Otherwise, find the closest point on the ball's line of travel...
    Geometry2d::Line line(position, position + velocity);
    Geometry2d::Point nearest = line.nearestPoint(near_to);

    double distance_to_nearest = (position - nearest).mag();
    double speed = velocity.mag();

    std::optional<RJ::Seconds> maybe_seconds = query_seconds_to_dist(distance_to_nearest);

    RJ::Seconds seconds;
    if (maybe_seconds.has_value()) {
        seconds = *maybe_seconds;
    } else {
        seconds = RJ::Seconds(std::numeric_limits<double>::infinity());
    }

    if (out != nullptr) {
        *out = position + velocity.normalized(speed * seconds.count() - 0.5 * kBallDecayConstant * std::pow(seconds.count(), 2));
    }

    return timestamp + seconds;
}

RJ::Seconds BallState::query_seconds_to(Geometry2d::Point target,
                                        Geometry2d::Point* out) const {
    return query_time_at(target, out) - timestamp;
}

RJ::Seconds BallState::query_stop_time() const {
    // Use the formula for time until zero velocity:
    // 0 = v = vi + at => t = -vi / a
    double speed = velocity.mag();
    return RJ::Seconds(speed / kBallDecayConstant);
}

std::optional<RJ::Seconds> BallState::query_seconds_to_dist(double distance) const {
    // vf^2 - vi^2 = 2ad => vf = sqrt(vi^2 + 2ad)
    double speed = velocity.mag();
    double vf_sq = std::pow(speed, 2) - 2 * kBallDecayConstant * distance;

    // If vf^2 is negative, the ball will never travel the desired distance.
    // Return nullopt.
    if (vf_sq < 0) {
        return std::nullopt;
    }

    // Otherwise, use t = (vf - vi) / a
    return RJ::Seconds(speed - std::sqrt(vf_sq)) / kBallDecayConstant;
}

Planning::Trajectory BallState::make_trajectory() {
    // The trajectory interface fits cubic splines. Luckily, a cubic spline
    // between two instants that can be connected by a constant acceleration
    // (like we have here) will be, and so we can use this for our trajectory.
    // The start point is the current instant in time, and the endpoint is the
    // stopping point.
    Planning::RobotInstant instant0;
    instant0.pose = Geometry2d::Pose(position, 0);
    instant0.velocity = Geometry2d::Twist(velocity, 0);
    instant0.stamp = timestamp;

    Planning::RobotInstant instant1;
    instant1.stamp = timestamp + query_stop_time();

    return Planning::Trajectory({instant0, instant1});
}
