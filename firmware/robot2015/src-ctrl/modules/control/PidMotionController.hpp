#pragma once

#include <array>
#include "MotionController.hpp"
#include "Pid.hpp"
#include "const-math.hpp"

class PidMotionController : public MotionController {
public:
    PidMotionController() {
        // TODO: set initial pid values to something sensible
    }

    void setTranslationalPidValues(float p, float i, float d) {
        // x controller
        _controllers[0].kp = p;
        _controllers[0].ki = i;
        _controllers[0].kd = d;

        // y controller
        _controllers[1].kp = p;
        _controllers[1].ki = i;
        _controllers[1].kd = d;
    }

    void setRotationalPidValues(float p, float i, float d) {
        _controllers[2].kp = p;
        _controllers[2].ki = i;
        _controllers[2].kd = d;
    }

    std::array<uint16_t, 4> run(std::array<float, 3> currVel) {

        std::array<float, 3> adjustedTargets;
        for (int i = 0; i < 3; ++i) {
            // steady-state control value
            adjustedTargets[i] = _ssGains[i] * _targetVel[i];

            // use controller to compensate for error
            float err = _targetVel[i] - currVel[i];
            adjustedTargets[i] += _controllers[i].run(err);
        }

        // TODO: get legit values for these
        // TODO: move to its own class to be shared with MotionController?
        const std::array<float, 4> wheel_angles = {M_PI / 4, 2*M_PI/4, 3*M_PI/4, 4*M_PI/4};


        std::array<uint16_t, 4> dutyCycles = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            // TODO: set duty cycle values based on @adjustedTargets
            // TODO: figure out wheel numbering
        }


        return dutyCycles;
    }

private:
    /// controllers for x, y, w
    Pid _controllers[3];
};
