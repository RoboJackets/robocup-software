#pragma once

#include "RobotModel.hpp"

#include <Eigen/Dense>

/**
 * Estimate robot velocity (x, y, w) from encoder readings
 */
class VelocityEstimator {
public:
    /**
     * @brief Calculate velocity from encoder ticks
     *
     * @param encoderDeltas Encoder deltas for the four drive motors
     * @param dt Time in ms since the last calll to update()
     *
     * @return x, y, w velocity in m/s
     */
    virtual std::array<float, 3> update(
        const std::array<uint16_t, 4>& encoderDeltas, uint16_t dt) {

        // TODO: how are negative encoder values formatted?

        std::array<float, 3> botVel = {0, 0, 0};

        for (int i = 0; i < 4; ++i) {
            // convert encoder ticks to rad/s
            const float wheelVel = encoderDeltas[i] / ENC_TICKS_PER_TURN * 2 * M_PI / dt;

            // calculate robot velocity from wheel velocities and wheel angles
            botVel[0] += wheelVel * cosf(RobotModel2015.WheelAngles[i]); // x vel
            botVel[1] += wheelVel * sinf(RobotModel2015.WheelAngles[i]); // y vel
            botVel[2] += wheelVel; // w vel
        }

        return botVel;
    }

    static const uint16_t ENC_TICKS_PER_TURN = 2048;
};
