#pragma once

// TODO: can this be included from somewhere?
const float PI = 3.14159265358979f;

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

        // TODO: get legit values for these
        // TODO: move to its own class to be shared with MotionController?
        const std::array<float, 4> wheel_angles = {PI / 4, 2*PI/4, 3*PI/4, 4*PI/4};

        std::array<float, 3> botVel = {0, 0, 0};

        for (int i = 0; i < 4; ++i) {
            // convert encoder ticks to rad/s
            const float wheelVel = encoderDeltas[i] / ENC_TICKS_PER_TURN * 2 * PI / dt;

            // calculate robot velocity from wheel velocities and wheel angles
            botVel[0] += wheelVel * cosf(wheel_angles[i]); // x vel
            botVel[1] += wheelVel * sinf(wheel_angles[i]); // y vel
            botVel[2] += wheelVel; // w vel
        }

        return botVel;
    }

    static const uint16_t ENC_TICKS_PER_TURN = 2048;
};
