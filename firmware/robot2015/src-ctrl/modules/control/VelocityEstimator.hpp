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

        // convert encoder ticks to rad/s
        std::array<float, 4> wheelRadPerSec;
        for (int i = 0; i < 4; ++i) {
            uint16_t encDelta = encoderDeltas[i];
            wheelRadPerSec[i] = encDelta / ENC_TICKS_PER_TURN * 2 * PI / dt;
        }

        // TODO: get legit values for these
        // TODO: move to its own class to be shared with MotionController?
        const std::array<float, 4> wheel_angles = {PI / 4, 2*PI/4, 3*PI/4, 4*PI/4};

        // calculate robot velocity from wheel velocities and wheel angles
        std::array<float, 3> vel = {0, 0, 0};
        for (int i = 0; i < 4; i++) {
            vel[0] += wheelRadPerSec[i] * cosf(wheel_angles[i]); // x vel
            vel[1] += wheelRadPerSec[i] * sinf(wheel_angles[i]); // y vel
            vel[2] += wheelRadPerSec[i]; // w vel
        }

        return vel;
    }

    static const uint16_t ENC_TICKS_PER_TURN = 2048;
};
