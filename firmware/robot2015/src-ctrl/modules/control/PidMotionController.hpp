#pragma once

#include <array>
#include "MotionController.hpp"
#include "Pid.hpp"
#include "RobotModel.hpp"

/**
 * Robot controller that runs a PID loop on each of the four wheels.
 */
class PidMotionController : public MotionController {
public:
    PidMotionController() {
        setPidValues(1, 0, 0);
    }

    void setPidValues(float p, float i, float d) {
        for (Pid& ctl : _controllers) {
            ctl.kp = p;
            ctl.ki = i;
            ctl.kd = d;
        }
    }

    /**
     * Return the duty cycle values for the motors to drive at the target
     * velocity.
     *
     * @param encoderDeltas Encoder deltas for the four drive motors
     * @param dt Time in ms since the last calll to run()
     *
     * @return Duty cycle values for each of the 4 motors
     */
    std::array<int16_t, 4> run(const std::array<int16_t, 4>& encoderDeltas,
                               float dt) {
        // convert encoder ticks to rad/s
        Eigen::Vector4f wheelVels;
        wheelVels << encoderDeltas[0], encoderDeltas[1], encoderDeltas[2],
            encoderDeltas[3];
        wheelVels *= ENC_TICKS_PER_TURN * 2 * M_PI / dt;

        Eigen::Vector4f targetWheelVels =
            RobotModel2015.BotToWheel * _targetVel;

        Eigen::Vector4f wheelVelErr = targetWheelVels - wheelVels;

        std::array<int16_t, 4> dutyCycles;
        for (int i = 0; i < 4; i++) {
            int16_t dc =
                targetWheelVels[i] * RobotModel2015.DutyCycleMultiplier;
            dc += _controllers[i].run(wheelVelErr[i]);

            dutyCycles[i] = dc;
        }

        return dutyCycles;
    }

    static const uint16_t ENC_TICKS_PER_TURN = 2048;

private:
    /// controllers for each wheel
    Pid _controllers[4];
};
