#include <spdlog/spdlog.h>

#include <rj_vision_filter/ball/camera_ball.hpp>

namespace vision_filter {
CameraBall::CameraBall(RJ::Time time_captured, const DetectionBallMsg& msg)
    : time_captured_{time_captured},
      pos_{rj_geometry::Point{msg.x / 1000, msg.y / 1000}} {}

RJ::Time CameraBall::get_time_captured() const { return time_captured_; }

rj_geometry::Point CameraBall::get_pos() const { return pos_; }

CameraBall CameraBall::combine_balls(const std::vector<CameraBall>& balls) {
    // Make sure we don't divide by zero due to some weird error
    if (balls.empty()) {
        SPDLOG_ERROR("Number of balls to combine is zero");

        return {RJ::now(), rj_geometry::Point(0, 0)};
    }

    // Have to do the average like Ti + sum(Tn - Ti)/N
    // so that we aren't trying to add time_points. It's durations instead.
    RJ::Time init_time = balls.front().get_time_captured();
    RJ::Seconds time_avg = RJ::Seconds(0);
    rj_geometry::Point pos_avg = rj_geometry::Point(0, 0);

    for (const CameraBall& ball : balls) {
        time_avg += RJ::Seconds(ball.get_time_captured() - init_time);
        pos_avg += ball.get_pos();
    }

    time_avg /= static_cast<double>(balls.size());
    pos_avg /= static_cast<double>(balls.size());

    return { init_time + time_avg, pos_avg };
}
}  // namespace vision_filter