#pragma once

#include <array>
#include "MotionController.hpp"
#include "RobotModel.hpp"
#include "Pid.hpp"

/**
 * Robot controller that runs a PID loop on each of the four wheels.
 */
class PidMotionController : public MotionController {
public:
    PidMotionController() {
        // TODO: set initial pid values to something more reasonable
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
     * @return x, y, w velocity in m/s
     */
    std::array<uint16_t, 4> run(const std::array<uint16_t, 4>& encoderDeltas, float dt) {

        // TODO: how are negative encoder values formatted?

        // convert encoder ticks to rad/s
        Eigen::Vector4f wheelVels;
        wheelVels << encoderDeltas[0],
                        encoderDeltas[1],
                        encoderDeltas[2],
                        encoderDeltas[3];
        wheelVels *= ENC_TICKS_PER_TURN * 2 * M_PI / dt;

        // Use @WheelToBot matrix to convert from wheel speeds to bot velocity
        // Eigen::Vector3f currentBotVel = RobotModel2015.WheelToBot * wheelVels;

        Eigen::Vector4f targetWheelVels = RobotModel2015.BotToWheel * _targetVel;

        Eigen::Vector4f wheelVelErr = targetWheelVels - wheelVels;

        std::array<uint16_t, 4> dutyCycles;
        for (int i = 0; i < 4; i++) {
            int16_t dc = targetWheelVels[i] * RobotModel2015.DutyCycleMultiplier;
            dc += _controllers[i].run(wheelVelErr[i]);

            // limit duty cycle, while keeping sign (+ or -)
            const uint16_t DUTY_CYCLE_MAX = 250;
            if (std::abs(dc) > DUTY_CYCLE_MAX) {
                dc = std::copysign(DUTY_CYCLE_MAX, dc);
            }

            // handle negative values - fpga expects them in sign-magnitude form
            if (dc < 0) {
                // TODO: double-check this
                dc = std::abs(dc);
                dc |= 1 << 16;
            }

            dutyCycles[i] = dc;
        }

        return dutyCycles;
    }


    static const uint16_t ENC_TICKS_PER_TURN = 2048;

private:
    /// controllers for each wheel
    Pid _controllers[4];
};
