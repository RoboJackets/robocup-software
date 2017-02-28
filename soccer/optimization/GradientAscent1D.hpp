#pragma once

#include "Gradient1DConfig.hpp"
#include <memory>

/**
 * Gradient Ascent in 1 Dimension with Temperature
 * Works for all functions that are continous with a specified derivative function
 * Only finds the closest local max
 *
 * Temperature controls X movement around discontinuity in the derivative function
 *      EX: X=0 when F(x) = abs(x)
 */
class GradientAscent1D {
public:
    GradientAscent1D(Gradient1DConfig* config);

    /**
     * Runs a single step of the optimization algorithm
     * 
     * @note This is most likely not the function you are looking for to find the
     *       max. This is used when executing multiple GradientAscent1D's in 
     *       parallel
     */
    bool singleStep();

    /**
     * Executes the full optimization algorithm
     * 
     * @note This function is used when there is no need to step through each
     *       iteration. Most uses of GradientAscent1D will call this function
     */
    void execute();

    /**
     * Returns a single X value of the current guess of the max
     */
    float getXValue();

    /**
     * Returns the current guess of the max value
     */
    float getValue();

    /**
     * Should continue execution?
     */
    bool continueExecution();

private:
    Gradient1DConfig* config;

    float currentVal;
    float currentx;
    float currentdx;
    float previousx;
    float previousdx;

    float temperature;

    int iterationCount;

    // Returns next x value
    float nextX();

    // Returns sign of value (-1, 0, 1)
    int sign(float val);
};