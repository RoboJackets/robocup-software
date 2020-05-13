#pragma once
// stores the outputs published by MotionControl
struct MotionSetpoint {
    double xvelocity;
    double yvelocity;
    double avelocity;
    void clear() { xvelocity = yvelocity = avelocity = 0; }
    MotionSetpoint() { clear(); }
};
