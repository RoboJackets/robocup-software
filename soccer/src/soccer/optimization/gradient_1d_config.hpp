#pragma once

#include <memory>
#include <tuple>
#include <functional>

/**
 * Config data for a Gradient Ascent 1D optimizer
 * Can be intitialized through the constructor or through
 * obj.attribute style initialization
 */
class Gradient1DConfig {
public:
    /**
     * Creates a Gradient Ascent 1D config
     * Default args: dx_error, max_x_movement, temperature_descent
     *               temperature_min, max_iterations, max_value
     *               max_thresh
     */
    Gradient1DConfig()
        : dx_error(0.1),
          max_x_movement(0.02),
          temperature_descent(0.5),
          temperature_min(0.01),
          max_iterations(100),
          max_value(0),
          max_thresh(0) {}

    /**
     * Creates a Gradient Ascent 1D config
     *
     * @param f, std function pointer which returns a tuple with <F(X), F'(X)>
     * @param start_x, X value in which to start from
     * @param prev_x, X value near start_x which is not star_tx
     * @param dx_error, threshold of F'(X) in reference
     *        to zero to exit
     * @param max_x_movement, max distance to go when
     *        stepping to either side in reference to current x
     * @param temperature_descent, factor by which to
     *        decrease the temperature each time we pass a
     *        discontinuity in the F'(X) space
     * @param temperature_min, minimum temperature to hit before
     *        exiting (Directly related to the X resolution)
     * @param max_iterations, Maximum number of iterations to reach end
     * @param max_value, used to leave function early when max value
     *        is already know (but corrseponding X is needed)
     * @param max_thresh, minimum distance to max before exit
     *
     * @note By default, the max value exit capabilities are not used
     * @note Only f, args, start_x, and prev_x are required
     */
    Gradient1DConfig(std::function<std::tuple<float, float>(float)>* f,
                     float start_x, float prev_x, float dx_error = 0.1,
                     float max_x_movement = 0.02, float temperature_descent = 0.5,
                     float temperature_min = 0.01, int max_iterations = 100,
                     float max_value = 0, float max_thresh = 0)
        :

          f(f),
          start_x(start_x),
          prev_x(prev_x),
          dx_error(dx_error),
          max_x_movement(max_x_movement),
          temperature_descent(temperature_descent),
          temperature_min(temperature_min),
          max_iterations(max_iterations),
          max_value(max_value),
          max_thresh(max_thresh) {}

    std::function<std::tuple<float, float>(float)>* f;
    float start_x;
    float prev_x;
    float dx_error;
    float max_x_movement;
    float temperature_descent;
    float temperature_min;
    int max_iterations;
    float max_value;
    float max_thresh;
};