#include "BallFilter.hpp"
#include "BallTracker.hpp"

#include <SystemState.hpp>

#include <stdio.h>

using namespace Geometry2d;

static const float Velocity_Alpha = 0.05;

BallFilter::BallFilter() {}

void BallFilter::update(const BallObservation* obs) {
    if (_estimate.time) {
        float dtime = (obs->time - _estimate.time) / 1000000.0f;
        Point newVel = (obs->pos - _estimate.pos) / dtime;
        _estimate.vel =
            newVel * Velocity_Alpha + _estimate.vel * (1.0f - Velocity_Alpha);
        _estimate.valid = true;
    } else {
        _estimate.vel = Point();
    }

    _estimate.pos = obs->pos;
    _estimate.time = obs->time;
}

void BallFilter::predict(RJ::Time time, Ball* out, float* velocityUncertainty) {
    if (velocityUncertainty) {
        *velocityUncertainty = 2 + _estimate.vel.mag() * 0.5;
    }

    if (out) {


        float t = RJ::TimestampToSecs(time - _estimate.time);

        const auto &vel = _estimate.vel;
        const auto &pos = _estimate.pos;
        const auto s0 = vel.mag();
        auto part = std::exp(-0.2913f*t);
        auto speed = s0 * part;
        auto distance = s0 *-3.43289f * (part - 1.0f);

        out->pos = pos + vel.normalized(distance);
        out->vel = vel.normalized(speed);
        //return MotionInstant(pos + vel.normalized(distance), vel.normalized(speed));
        //out->pos = _estimate.pos +
        //           _estimate.vel * (time - _estimate.time) / 1000000.0f;
        //out->vel = _estimate.vel;
        out->time = time;
        out->valid = true;
    }
}
