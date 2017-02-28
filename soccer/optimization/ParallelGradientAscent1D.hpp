#pragma once

#include "GradientAscent1D.hpp"
#include "ParallelGradient1DConfig.hpp"
#include <vector>

/**
 * Starts multiple "Gradient Ascent 1D" (GA1D) at various start points
 * Combines two single GA1D's together when they
 * are near the same X value
 */
class ParallelGradientAscent1D {
public:
    ParallelGradientAscent1D(ParallelGradient1DConfig* config);

    /**
     * Executes all GA1Ds until their max has been reached
     */
    void execute();

    /**
     * Returns a list of all X values for each max in ascending order
     */
    std::vector<float> getMaxXValues();

    /**
     * Returns a list of all the values for each max in ascending X order
     */
    std::vector<float> getMaxValues();

private:
    ParallelGradient1DConfig* config;

    std::vector<GradientAscent1D> GA1Ds;
};