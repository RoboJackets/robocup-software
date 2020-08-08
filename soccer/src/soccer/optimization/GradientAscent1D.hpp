#pragma once

#include "Gradient1DConfig.hpp"
#include <memory>

/**
 * Gradient Ascent in 1 Dimension with Temperature
 * Works for all functions that are continous with a specified derivative
 * function
 * Only finds the closest local max
 *
 * Temperature controls X movement around discontinuity in the derivative
 * function
 *      EX: X=0 when F(x) = abs(x)
 *
 * Use Example:
 * GradientAscent1D ga(& [Gradient1DConfig]);
 * ga.execute();
 * ga.get_value();
 */
class GradientAscent1D {
public:
    GradientAscent1D(Gradient1DConfig* config);

    /**
     * Runs a single step of the optimization algorithm
     *
     * @note This is most likely not the function you are looking for to find
     *       the max. This is used when executing multiple GradientAscent1D's
     *       in parallel
     */
    bool single_step();

    /**
     * Executes the full optimization algorithm
     *
     * @note This function is used when there is no need to step through each
     *       iteration. Most uses of GradientAscent1D will call this function
     */
    void execute();

    /**
     * @return the X value of the current guess of the max
     */
    float get_x_value();

    /**
     * @return the current guess of the max value
     */
    float get_value();

    /**
     * @return Should continue execution?
     */
    bool continue_execution();

private:
    Gradient1DConfig* config_;

    float current_val_;
    float currentx_;
    float currentdx_;
    float previousx_;
    float previousdx_;

    float temperature_;

    int iteration_count_;

    /**
     * @return next x value based on derivative
     */
    float next_x();
};