#pragma once

#include "FunctionArgs.hpp"
#include <memory>
#include <tuple>

class Gradient1DConfig {
public:
    /**
     * Creates a Gradient Ascent 1D config
     *
     * @param f, function pointer which returns a tuple with <F(X), F'(X)>
     * @param args, F(X) args, Pointer to class for the specific 
     *        F(X) and F'(X) arguements
     * @param startX, X value in which to start from
     * @param prevX, X value near startX which is not startX
     * @param dxError, threshold of F'(X) in reference 
     *        to zero to exit
     * @param maxXMovement, max distance to go when 
     *        stepping to either side in reference to current x
     * @param temperatureDescent, factor by which to 
     *        decrease the temperature each time we pass a 
     *        discontinuity in the F'(X) space
     * @param temperatureMin, minimum temperature to hit before 
     *        exiting (Directly related to the X resolution)
     * @param maxIterations, Maximum number of iterations to reach end
     * @param maxValue, used to leave function early when max value 
     *        is already know (but corrseponding X is needed)
     * @param maxThresh, minimum distance to max before exit
     *
     * @note By default, the max value exit capabilities are not used
     * @note Only f, args, startX, and prevX are required
     */
    Gradient1DConfig(std::tuple<double, double> (*f) (double, FunctionArgs*),  
                     FunctionArgs* args, double startX, double prevX, 
                     double dxError = 0.1, double maxXMovement = 0.02, 
                     double temperatureDescent = 0.5, double temperatureMin = 0.01, 
                     int maxIterations = 100, double maxValue = 0, double maxThresh = 0) : 
                     f(f), args(args), startX(startX), prevX(prevX), dxError(dxError), 
                     maxXMovement(maxXMovement), temperatureDescent(temperatureDescent),
                     temperatureMin(temperatureMin), maxIterations(maxIterations), 
                     maxValue(maxValue), maxThresh(maxThresh) {}

    std::tuple<double, double> (*f) (double, FunctionArgs*);
    FunctionArgs* args;
    double startX;
    double prevX;
    double dxError;
    double maxXMovement;
    double temperatureDescent;
    double temperatureMin;
    int    maxIterations;
    double maxValue;
    double maxThresh;
};