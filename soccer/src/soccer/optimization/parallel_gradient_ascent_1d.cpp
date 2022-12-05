#include "parallel_gradient_ascent_1d.hpp"

#include <algorithm>

#include <math.h>

ParallelGradientAscent1D::ParallelGradientAscent1D(ParallelGradient1DConfig* config)
    : config_(config) {
    problems_.reserve(config_->ga_config.size());

    // Create list of problems
    for (size_t i = 0; i < config_->ga_config.size(); i++) {
        problems_.push_back(GradientAscent1D(&config_->ga_config.at(i)));
    }
}

/**
 * Executes all problems until their max has been reached
 */
void ParallelGradientAscent1D::execute() {
    // While any are not done
    bool continue_execution = true;

    while (continue_execution) {
        // Default to false unless any still need to work
        continue_execution = false;

        // Execute a step for each one
        for (auto& problem : problems_) {
            if (problem.continue_execution()) {
                problem.single_step();
                continue_execution = true;
            }
        }

        // Assume ascending order for x_start
        // Remove any that are too close
        for (size_t i = 0; i < problems_.size() - 1; i++) {
            GradientAscent1D lower = problems_.at(i);
            GradientAscent1D upper = problems_.at(i + 1);

            // Erase elements if they get too close
            // This helps kill any problems_ that are going up the same hill
            if (fabs(lower.get_x_value() - upper.get_x_value()) < config_->x_combine_thresh) {
                problems_.erase(problems_.begin() + i + 1);
            }
        }
    }
}

/**
 * Returns a list of all X values for each max in ascending order
 */
std::vector<float> ParallelGradientAscent1D::get_max_x_values() {
    std::vector<float> x_vals;
    x_vals.reserve(problems_.size());

    for (auto& problem : problems_) {
        x_vals.push_back(problem.get_x_value());
    }

    return x_vals;
}

/**
 * Returns a list of all X values for each max in ascending order
 */
std::vector<float> ParallelGradientAscent1D::get_max_values() {
    std::vector<float> vals;
    vals.reserve(problems_.size());

    for (auto& problem : problems_) {
        vals.push_back(problem.get_value());
    }

    return vals;
}