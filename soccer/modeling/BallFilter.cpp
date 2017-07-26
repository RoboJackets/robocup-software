#include "BallFilter.hpp"
#include "BallTracker.hpp"

#include <SystemState.hpp>

#include <stdio.h>

using namespace Geometry2d;

static const float Velocity_Alpha = 0.05;

BallFilter::BallFilter() {}

void BallFilter::updateEstimate(const BallObservation& obs) {
    RJ::Seconds dtime = (obs.time - _currentEstimate.time);
    Point newVel = (obs.pos - _currentEstimate.pos) / dtime.count();

    _currentEstimate.valid = true;
    _currentEstimate.pos = obs.pos;
    _currentEstimate.vel = newVel * Velocity_Alpha +
                           _currentEstimate.vel * (1.0f - Velocity_Alpha);
    _currentEstimate.time = obs.time;
}

Ball BallFilter::predict(RJ::Time time, float* velocityUncertainty) const {
    Ball prediction{};

    RJ::Seconds t = time - _currentEstimate.time;

    const auto& vel = _currentEstimate.vel;
    const auto& pos = _currentEstimate.pos;
    const auto s0 = vel.mag();

//    const auto decayConstant = 0.1735;
    const auto decayConstant = 0.1795;


//    auto part = std::exp(-0.2913f * t.count());
    // vel = v0 - t*Constant
    // 0 = v0 - t * C
    // t*C = v0
    // t = v0/C

    // d = v0t - 1/2*t^2*Constant

    double speed=0;
    double distance=0;
    if (s0 != 0) {
        auto maxTime = s0/decayConstant;
        if (t.count() >= maxTime) {
            speed = 0;
            distance = s0 * maxTime - pow(maxTime, 2) / 2.0 * decayConstant;
        } else {
            speed = s0 - (t.count() * decayConstant);
            distance = s0 * t.count() - pow(t.count(), 2) / 2.0 * decayConstant;
        }
    } else {
        speed = 0;
        distance = 0;
    }

    prediction.time = time;
    prediction.valid = true;
    prediction.pos = pos + vel.normalized(distance);
    prediction.vel = vel.normalized(speed);

    if (velocityUncertainty) {
        *velocityUncertainty = 2.0f + _currentEstimate.vel.mag() * 0.5f;
    }

    return prediction;
}
