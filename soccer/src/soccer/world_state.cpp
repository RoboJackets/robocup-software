#include "world_state.hpp"

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

    double speed = 0.0;
    double distance = 0.0;

    double max_time = s0 / soccer::physics::PARAM_ball_decay_constant;
    if (dt.count() >= max_time) {
        speed = 0;
        distance = s0 * max_time - pow(max_time, 2) / 2.0 * soccer::physics::PARAM_ball_decay_constant;
    } else {
        speed = s0 - (dt.count() * soccer::physics::PARAM_ball_decay_constant);
        distance = s0 * dt.count() - pow(dt.count(), 2) / 2.0 * soccer::physics::PARAM_ball_decay_constant;
    }

    return BallState(position + velocity.normalized(distance), velocity.normalized(speed), time);
}

BallState BallState::predict_in(RJ::Seconds seconds) const {
    return predict_at(timestamp + seconds);
}

RJ::Time BallState::query_time_near(rj_geometry::Point near_to, rj_geometry::Point* out) const {
    // If the ball isn't moving we're as close as we're ever going to get.
    if (velocity.mag() == 0) {
        if (out != nullptr) {
            *out = position;
        }
        return timestamp;
    }

    // Otherwise, find the closest point on the ball's line of travel...
    rj_geometry::Segment segment(position, predict_at(RJ::Time::max()).position);
    rj_geometry::Point nearest = segment.nearest_point(near_to);

    double distance_to_nearest = (position - nearest).mag();

    std::optional<RJ::Seconds> maybe_seconds = query_seconds_to_dist(distance_to_nearest);

    RJ::Seconds seconds;
    if (maybe_seconds.has_value()) {
        seconds = *maybe_seconds;
    } else {
        seconds = query_stop_time();
    }

    if (out != nullptr) {
        *out = predict_in(seconds).position;
    }

    return timestamp + seconds;
}

RJ::Seconds BallState::query_seconds_near(rj_geometry::Point near_to, rj_geometry::Point* out) const {
    return query_time_near(near_to, out) - timestamp;
}

RJ::Seconds BallState::query_stop_time(rj_geometry::Point* out) const {
    double speed = velocity.mag();

    if (out != nullptr) {
        // vf^2 - vi^2 = 2ad => d = -vi^2 / 2a
        *out = position + velocity.normalized(std::pow(speed, 2) / (2 * soccer::physics::PARAM_ball_decay_constant));
    }

    // Use the formula for time until zero velocity:
    // 0 = v = vi + at => t = -vi / a
    return RJ::Seconds(speed / soccer::physics::PARAM_ball_decay_constant);
}

rj_geometry::Point BallState::query_stop_position() const {
    rj_geometry::Point point;
    [[maybe_unused]] auto stop_time = query_stop_time(&point);
    return point;
}

std::optional<RJ::Seconds> BallState::query_seconds_to_dist(double distance) const {
    // vf^2 - vi^2 = 2ad => vf = sqrt(vi^2 + 2ad)
    double speed = velocity.mag();
    double vf_sq = std::pow(speed, 2) - 2 * soccer::physics::PARAM_ball_decay_constant * distance;

    // If vf^2 is negative, the ball will never travel the desired distance.
    // Return nullopt.
    if (vf_sq < 0) {
        return std::nullopt;
    }

    // Otherwise, use t = (vf - vi) / a
    return RJ::Seconds(speed - std::sqrt(vf_sq)) / soccer::physics::PARAM_ball_decay_constant;
}

planning::Trajectory BallState::make_trajectory() const {
    using namespace rj_geometry;

    // The trajectory interface fits cubic splines. Luckily, a cubic spline
    // between two instants that can be connected by a constant acceleration
    // (like we have here) will be, and so we can use this for our trajectory.
    // The start point is the current instant in time, and the endpoint is the
    // stopping point.
    planning::RobotInstant instant0;
    instant0.pose = Pose(position, 0);
    instant0.velocity = Twist(velocity, 0);
    instant0.stamp = timestamp;

    Point stop_position;
    RJ::Time stop_time = timestamp + query_stop_time(&stop_position);
    planning::RobotInstant instant1{Pose{stop_position, 0}, Twist::zero(), stop_time};

    return planning::Trajectory({instant0, instant1});
}
