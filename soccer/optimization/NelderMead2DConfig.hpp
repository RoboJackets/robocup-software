#pragma once

#include <geometry2d/point.h>

#include <functional>

class NelderMead2DConfig {
public:
    /**
     * Creates a Nelder-Mead 2D Config
     *
     * @param f std function pointer which returns F(X, Y)
     * @param start starting point of the simplex (Triangle in 2D Case)
     * @param step starting step magnitudes in X, Y directions
     * @param minDist minimum distance of bounding box before exit
     * @param reflectionCoeff perecent to reflect by in the oposite direction
     *           Must be greater than 0
     * @param expansionCoeff percent to extend single point by
     *           Must be greater than 1
     * @param contractionCoeff percent to contract single point by
     *           Must be greater than 0 and less than or equal to 0.5
     * @param shrinkCoeff percent to shrink all points by
     *           Must be between 0 and 1
     * @param maxIterations maximum number of iterations to reach before end
     * @param maxValue max value to exit early at
     * @param maxThresh threshold for the max vlaue before exit
     * @note Set maxValue = maxThresh to disable
     */
    NelderMead2DConfig(std::function<float(geometry2d::Point)>& f,
                       geometry2d::Point start = geometry2d::Point(0, 0),
                       geometry2d::Point step = geometry2d::Point(1, 1),
                       geometry2d::Point minDist = geometry2d::Point(0.001,
                                                                     0.001),
                       float reflectionCoeff = 1, float expansionCoeff = 2,
                       float contractionCoeff = 0.5, float shrinkCoeff = 0.5,
                       int maxIterations = 100, float maxValue = 0,
                       float maxThresh = 0)
        : f(f),
          start(start),
          step(step),
          minDist(minDist),
          reflectionCoeff(reflectionCoeff),
          expansionCoeff(expansionCoeff),
          contractionCoeff(contractionCoeff),
          shrinkCoeff(shrinkCoeff),
          maxIterations(maxIterations),
          maxValue(maxValue),
          maxThresh(maxThresh) {}

    std::function<float(geometry2d::Point)>& f;
    geometry2d::Point start;
    geometry2d::Point step;
    geometry2d::Point minDist;
    float reflectionCoeff;
    float expansionCoeff;
    float contractionCoeff;
    float shrinkCoeff;
    int maxIterations;
    float maxValue;
    float maxThresh;
};
