#pragma once

#include "Gradient1DConfig.hpp"
#include <vector>
#include <memory>

/**
 * Config data for a Parallel Gradient 1D optimizer
 * Can be intitialized through the constructor or through
 * obj.attribute style initialization
 */
class ParallelGradient1DConfig {
public:
    /**
     * Default constructor
     * GA1DConfig is initialized to empty
     * xCombineThresh is initialized to 0.1
     */
    ParallelGradient1DConfig() : xCombineThresh(0.1) {}

    /**
     * Creates a Parallel Gradient Ascent 1D config
     *
     * @param GA1DConfig, vector of GA1D configs, xStart must be ascending
     * @param xCombineThresh, Minimum delta X before two GA1D's are combined
     */
    ParallelGradient1DConfig(std::vector<Gradient1DConfig> GA1DConfig,
                             float xCombineThresh)
        :

          GA1DConfig(std::move(GA1DConfig)),
          xCombineThresh(xCombineThresh) {}

    std::vector<Gradient1DConfig> GA1DConfig;
    float xCombineThresh;
};