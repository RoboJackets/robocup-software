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

        for (auto& ctrl : _controllers) {
            ctrl.setWindup(5);
        }
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
        wheelVels *= 2 * M_PI / ENC_TICKS_PER_TURN / dt;

        Eigen::Vector4f targetWheelVels =
            RobotModel2015.BotToWheel * _targetVel;

        // printf("wheelVels[0] = %f\r\n", wheelVels[0]);

        // printf("targetWheelVels: %f, %f, %f, %f\r\n", targetWheelVels[0], targetWheelVels[1], targetWheelVels[2], targetWheelVels[3]);

        Eigen::Vector4f wheelVelErr = targetWheelVels - wheelVels;

        // printf("wheelVelErr:\r\n  ");
        // for (int i = 0; i < 4; i++) {
        //     printf("%f, ", wheelVelErr[i]);
        // }
        // printf("\r\n");

        std::array<int16_t, 4> dutyCycles;
        for (int i = 0; i < 4; i++) {
            int16_t dc =
                targetWheelVels[i] * RobotModel2015.DutyCycleMultiplier;
            dc += _controllers[i].run(wheelVelErr[i]);

            dutyCycles[i] = dc;
        }

        // printf("pid.run(), target.x = %f\r\n", _targetVel[0]);

        return dutyCycles;
    }

    static const uint16_t ENC_TICKS_PER_TURN = 2048;

private:
    /// controllers for each wheel
    Pid _controllers[4];
};






