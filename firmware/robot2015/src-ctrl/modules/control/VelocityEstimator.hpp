
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
        // TODO: implement
        return {0, 0, 0};
    }
};
