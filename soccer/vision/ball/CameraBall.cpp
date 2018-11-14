#include "CameraBall.hpp"

RJ::Time CameraBall::getTimeCaptured() {
    return timeCaptured;
}

Geometry2d::Point CameraBall::getPos() {
    return pos;
}

CameraBall CameraBall::CombineBalls(std::vector<CameraBall> balls) {
    RJ::Time timeAvg = 0;
    Geometry2d::Point posAvg = Geometry2d::Point(0,0);

    for (CameraBall &cb : balls) {
        timeAvg += cb.getTimeCaptured();
        posAvg += cb.getPos();
    }

    timeAvg /= balls.size();
    posAvg /= balls.size();

    return CameraBall(timeAvg, posAvg);
}