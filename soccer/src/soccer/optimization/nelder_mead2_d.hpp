#pragma once

#include "nelder_mead2_d_config.hpp"
#include <Geometry2d/Point.hpp>
#include <vector>
#include <tuple>

/**
 * Nelder-Mead in 2 Dimensions
 * Works for all functions that are continous
 * Finds local / global max depending on size of the simplex
 * Can be extended to work in N dimensions
 *
 * Use Example:
 * NelderMead2D nm(& [NelderMead2DConfig]);
 * nm.execute();
 * nm.get_value();
 */

class NelderMead2D {
public:
    NelderMead2D(NelderMead2DConfig& config);

    /**
     * Runs a single step of the optimization algorithm
     * (One reflection / expansion / contraction / shrink)
     *
     * @note This is most ikely not the function you are looking for to find
     *       the max. This is used when you want to split the optimization
     *       accross multiple time steps
     */
    bool single_step();

    /**
     * Executes the full optimization algorithm
     */
    void execute();

    /**
     * @return the XY coordinate of the current guess of the max
     */
    Geometry2d::Point get_point();

    /**
     * @return the current guess of the max value
     */
    float get_value();

    /**
     * @return Should continue execution?
     */
    bool continue_execution();

private:
    NelderMead2DConfig& config_;
    int iteration_count_;
    std::vector<std::tuple<float, Geometry2d::Point>> vertices_;

    void sort_vertices();
    bool replace_worst(float new_score, Geometry2d::Point new_point);
};