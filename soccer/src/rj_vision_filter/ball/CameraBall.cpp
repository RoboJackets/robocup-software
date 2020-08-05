#include <rj_vision_filter/ball/CameraBall.hpp>

namespace vision_filter {
CameraBall::CameraBall(RJ::Time time_captured, const DetectionBallMsg& msg,
                       const Geometry2d::TransformMatrix& world_to_team)
    : timeCaptured{time_captured},
      pos{world_to_team * Geometry2d::Point{msg.x / 1000, msg.y / 1000}} {}

RJ::Time CameraBall::getTimeCaptured() const { return timeCaptured; }

Geometry2d::Point CameraBall::getPos() const { return pos; }

CameraBall CameraBall::CombineBalls(const std::vector<CameraBall>& balls) {
    // Make sure we don't divide by zero due to some weird error
    if (balls.empty()) {
        std::cout << "ERROR: Number of balls to combine is zero" << std::endl;

        return CameraBall(RJ::now(), Geometry2d::Point(0, 0));
    }

    // Have to do the average like Ti + sum(Tn - Ti)/N
    // so that we aren't trying to add time_points. It's durations instead.
    RJ::Time init_time = balls.front().getTimeCaptured();
    RJ::Seconds time_avg = RJ::Seconds(0);
    Geometry2d::Point pos_avg = Geometry2d::Point(0, 0);

    for (const CameraBall& cb : balls) {
        time_avg += RJ::Seconds(cb.getTimeCaptured() - init_time);
        pos_avg += cb.getPos();
    }

    time_avg /= balls.size();
    pos_avg /= balls.size();

    return CameraBall(init_time + time_avg, pos_avg);
}
}  // namespace vision_filter