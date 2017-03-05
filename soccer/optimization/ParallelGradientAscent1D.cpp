#include "ParallelGradientAscent1D.hpp"
#include <math.h>
#include <algorithm>

ParallelGradientAscent1D::ParallelGradientAscent1D(
    ParallelGradient1DConfig* config)
    : config(config) {
    GA1Ds.reserve(config->GA1DConfig.size());

    // Create list of GA1Ds
    for (int i = 0; i < config->GA1DConfig.size(); i++) {
        GA1Ds.push_back(GradientAscent1D(&config->GA1DConfig.at(i)));
    }
}

/**
 * Executes all GA1Ds until their max has been reached
 */
void ParallelGradientAscent1D::execute() {
    // While any are not done
    bool continueExecution = true;

    while (continueExecution) {
        // Default to false unless any still need to work
        continueExecution = false;

        // Execute a step for each one
        for (auto& GA1D : GA1Ds) {
            if (GA1D.continueExecution()) {
                GA1D.singleStep();
                continueExecution = true;
            }
        }

        // Assume ascending order for xStart
        // Remove any that are too close
        int i = 1;
        GA1Ds.erase(
            std::remove_if(GA1Ds.begin(), GA1Ds.end(), [this, &i](auto& value) {
                auto& next = GA1Ds[i];
                i++;
                return fabs(value.getXValue() - next.getXValue()) <
                       config->xCombineThresh;
            }), GA1Ds.end());
    }
}

/**
 * Returns a list of all X values for each max in ascending order
 */
std::vector<float> ParallelGradientAscent1D::getMaxXValues() {
    std::vector<float> xVals;
    xVals.reserve(GA1Ds.size());

    for (auto& GA1D : GA1Ds) {
        xVals.push_back(GA1D.getXValue());
    }

    return xVals;
}

/**
 * Returns a list of all X values for each max in ascending order
 */
std::vector<float> ParallelGradientAscent1D::getMaxValues() {
    std::vector<float> vals;
    vals.reserve(GA1Ds.size());

    for (auto& GA1D : GA1Ds) {
        vals.push_back(GA1D.getValue());
    }

    return vals;
}