#pragma once

#include "Gradient1DConfig.hpp"
#include <vector>
#include <memory>

class ParallelGradient1DConfig {
public:
    /**
     * Creates a Parallel Gradient Ascent 1D config
     *
     * @param GA1DConfig, vector of GA1D configs, xStart must be ascending
     * @param xCombineThresh, Minimum delta X before two GA1D's are combined 
     */
    ParallelGradient1DConfig(std::vector<Gradient1DConfig> GA1DConfig,
                             float xCombineThresh) : GA1DConfig(std::move(GA1DConfig)), 
                             xCombineThresh(xCombineThresh) {}

    std::vector<Gradient1DConfig> GA1DConfig;
    float xCombineThresh;
};