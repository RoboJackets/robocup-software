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
    auto part = std::exp(-0.2913f * t.count());
    auto speed = s0 * part;
    auto distance = s0 * -3.43289f * (part - 1.0f);

    prediction.time = time;
    prediction.valid = true;
    prediction.pos = pos + vel.normalized(distance);
    prediction.vel = vel.normalized(speed);

    if (velocityUncertainty) {
        *velocityUncertainty = 2.0f + _currentEstimate.vel.mag() * 0.5f;
    }
    return prediction;
}
