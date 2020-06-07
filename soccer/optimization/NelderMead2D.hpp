#pragma once

#include <geometry2d/point.h>

#include <tuple>
#include <vector>

#include "NelderMead2DConfig.hpp"

/**
 * Nelder-Mead in 2 Dimensions
 * Works for all functions that are continous
 * Finds local / global max depending on size of the simplex
 * Can be extended to work in N dimensions
 *
 * Use Example:
 * NelderMead2D nm(& [NelderMead2DConfig]);
 * nm.execute();
 * nm.getValue();
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
    bool singleStep();

    /**
     * Executes the full optimization algorithm
     */
    void execute();

    /**
     * @return the XY coordinate of the current guess of the max
     */
    geometry2d::Point getPoint();

    /**
     * @return the current guess of the max value
     */
    float getValue();

    /**
     * @return Should continue execution?
     */
    bool continueExecution();

private:
    NelderMead2DConfig& config;
    int iterationCount;
    std::vector<std::tuple<float, geometry2d::Point>> vertices;

    void sortVertices();
    bool replaceWorst(float newScore, geometry2d::Point newPoint);
};
