#pragma once
//stores the outputs published by MotionControl
struct MotionSetpoint {
    float xvelocity;
    float yvelocity;
    float avelocity;
    void clear() { xvelocity = yvelocity = avelocity = 0; }
    MotionSetpoint() { clear(); }
};