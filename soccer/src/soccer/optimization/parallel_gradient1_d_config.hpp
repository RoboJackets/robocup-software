#pragma once

#include "gradient1_d_config.hpp"
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
     * x_combine_thresh is initialized to 0.1
     */
    ParallelGradient1DConfig() : x_combine_thresh(0.1) {}

    /**
     * Creates a Parallel Gradient Ascent 1D config
     *
     * @param GA1DConfig, vector of GA1D configs, x_start must be ascending
     * @param x_combine_thresh, Minimum delta X before two GA1D's are combined
     */
    ParallelGradient1DConfig(std::vector<Gradient1DConfig> ga_config,
                             float x_combine_thresh)
        :

          ga_config(std::move(ga_config)),
          x_combine_thresh(x_combine_thresh) {}

    std::vector<Gradient1DConfig> ga_config;
    float x_combine_thresh;
};