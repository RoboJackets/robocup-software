#include "CameraBall.hpp"

RJ::Time CameraBall::getTimeCaptured() const {
    return timeCaptured;
}

Geometry2d::Point CameraBall::getPos() const {
    return pos;
}

CameraBall CameraBall::CombineBalls(const std::vector<CameraBall>& balls) {
    // Make sure we don't divide by zero due to some weird error
    if (balls.size() == 0) {
        std::cout << "ERROR: Number of balls to combine is zero" << std::endl;

        return CameraBall(RJ::now(), Geometry2d::Point(0,0));
    }

    // Have to do the average like Ti + sum(Tn - Ti)/N
    // so that we aren't trying to add time_points. It's durations instead.
    RJ::Time initTime = balls.front().getTimeCaptured();
    RJ::Seconds timeAvg = RJ::Seconds(0);
    Geometry2d::Point posAvg = Geometry2d::Point(0,0);

    for (const CameraBall& cb : balls) {
        timeAvg += RJ::Seconds(cb.getTimeCaptured() - initTime);
        posAvg += cb.getPos();
    }

    timeAvg /= balls.size();
    posAvg /= balls.size();

    return CameraBall(initTime + timeAvg, posAvg);
}