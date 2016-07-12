#pragma once

#include <SystemState.hpp>

#include <stdint.h>

class Ball;
class BallObservation;

// A BallFilter never needs to reset itself.  The BallTracker will
// create a new one when a new ball is found.
class BallFilter {
public:
    BallFilter();

    // Gives a new observation to the filter
    void update(const BallObservation* obs);

    // Generates a prediction of the ball's state at a given time in the future
    void predict(RJ::Time time, Ball* out, float* velocityUncertainty);

private:
    Ball _estimate;
};
