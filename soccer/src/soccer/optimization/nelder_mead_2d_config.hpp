#pragma once

#include <rj_geometry/point.hpp>
#include <functional>

class NelderMead2DConfig {
public:
    /**
     * Creates a Nelder-Mead 2D Config
     *
     * @param f std function pointer which returns F(X, Y)
     * @param start starting point of the simplex (Triangle in 2D Case)
     * @param step starting step magnitudes in X, Y directions
     * @param min_dist minimum distance of bounding box before exit
     * @param reflection_coeff perecent to reflect by in the oposite direction
     *           Must be greater than 0
     * @param expansion_coeff percent to extend single point by
     *           Must be greater than 1
     * @param contraction_coeff percent to contract single point by
     *           Must be greater than 0 and less than or equal to 0.5
     * @param shrink_coeff percent to shrink all points by
     *           Must be between 0 and 1
     * @param max_iterations maximum number of iterations to reach before end
     * @param max_value max value to exit early at
     * @param max_thresh threshold for the max vlaue before exit
     * @note Set max_value = max_thresh to disable
     */
    NelderMead2DConfig(std::function<float(rj_geometry::Point)>& f,
                       rj_geometry::Point start = rj_geometry::Point(0, 0),
                       rj_geometry::Point step = rj_geometry::Point(1, 1),
                       rj_geometry::Point min_dist = rj_geometry::Point(0.001,
                                                                     0.001),
                       float reflection_coeff = 1, float expansion_coeff = 2,
                       float contraction_coeff = 0.5, float shrink_coeff = 0.5,
                       int max_iterations = 100, float max_value = 0,
                       float max_thresh = 0)
        : f(f),
          start(start),
          step(step),
          min_dist(min_dist),
          reflection_coeff(reflection_coeff),
          expansion_coeff(expansion_coeff),
          contraction_coeff(contraction_coeff),
          shrink_coeff(shrink_coeff),
          max_iterations(max_iterations),
          max_value(max_value),
          max_thresh(max_thresh) {}

    std::function<float(rj_geometry::Point)>& f;
    rj_geometry::Point start;
    rj_geometry::Point step;
    rj_geometry::Point min_dist;
    float reflection_coeff;
    float expansion_coeff;
    float contraction_coeff;
    float shrink_coeff;
    int max_iterations;
    float max_value;
    float max_thresh;
};