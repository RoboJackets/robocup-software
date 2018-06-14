#include "BallFilter.hpp"
#include "BallTracker.hpp"

#include <SystemState.hpp>

#include <stdio.h>

using namespace Geometry2d;

static const float Velocity_Alpha = 0.2;

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
    const auto& estimate = _currentEstimate.predict(time);

    prediction.time = time;
    prediction.valid = true;
    prediction.pos = estimate.pos;
    prediction.vel = estimate.vel;

    return prediction;
}
