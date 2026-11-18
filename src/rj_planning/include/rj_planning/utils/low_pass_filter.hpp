#pragma once

namespace planning {

/*
 * @brief Approximate moving average with first-order low pass filter.
 *
 * @details For those without a digital signal processing background, think of
 * it as giving small weights to new estimates compared to the existing belief.
 * For example, to approximate the avg velocity, with a gain of 0.8, we can do:
 *      new_avg_vel = (0.8 * old_avg_vel) + (0.2 * most_recent_vel)
 * every time a new velocity comes in. This protects us from noisy velocity
 * readings while also updating the average as it changes.
 *
 * @param old_value current value of moving average
 * @param new_value new value to add to moving average
 * @param gain resistance to noise (higher = more weight to old_value)
 * @return new moving average estimate
 */
template <typename T>
inline T apply_low_pass_filter(const T& old_value, const T& new_value, double gain) {
    return gain * new_value + (1 - gain) * old_value;
}

}  // namespace planning
