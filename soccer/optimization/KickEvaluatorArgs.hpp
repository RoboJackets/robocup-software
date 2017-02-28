#pragma once

#include "FunctionArgs.hpp"
#include <vector>

/**
 * Contains the arguements for the KickEvaluator function used in optimization
 */
class KickEvaluatorArgs : public FunctionArgs {
public:
    KickEvaluatorArgs() {}
    /**
     * Contains the static args for F(x) and F'(X)
     *
     * @param kickMean, mean of the kick
     * @param kickStDev, standard deviation of the kick
     * @param robotMeans, mean of the robot position distribution
     * @param robotStDevs, standard deviation of the position distribution
     * @param robotVertScales, Scales height of distribution when robot is
     *        behind the target segment (Units: % of original)
     * @param boundaryLower, left boundary
     * @param boundaryUpper, right boundary
     *
     * @note All arguements are in radians in reference to the center of
     *       the kick target segment (Unless otherwise specified)
     */
    KickEvaluatorArgs(float kickMean, float kickStDev, 
                      std::vector<float> robotMeans,
                      std::vector<float> robotStDevs,
                      std::vector<float> robotVertScales,
                      float boundaryLower, float boundaryUpper) :
                      kickMean(kickMean), kickStDev(kickStDev),
                      robotMeans(robotMeans), robotStDevs(robotStDevs),
                      robotVertScales(robotVertScales),
                      boundaryLower(boundaryLower), 
                      boundaryUpper(boundaryUpper) {}

    float kickMean;
    float kickStDev;

    std::vector<float> robotMeans;
    std::vector<float> robotStDevs;
    std::vector<float> robotVertScales;

    float boundaryLower;
    float boundaryUpper;
};