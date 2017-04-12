#pragma once

#include <memory>
#include <tuple>
#include <functional>

/**
 * Config data for a Gradient Ascent 1D optimizer
 * Can be intitialized through the constructor or through
 * obj.attribute style initialization
 */
class Gradient1DConfig {
public:
    /**
     * Creates a Gradient Ascent 1D config
     * Default args: dxError, maxXMovement, temperatureDescent
     *               temperatureMin, maxIterations, maxValue
     *               maxThresh
     */
    Gradient1DConfig()
        : dxError(0.1),
          maxXMovement(0.02),
          temperatureDescent(0.5),
          temperatureMin(0.01),
          maxIterations(100),
          maxValue(0),
          maxThresh(0) {}

    /**
     * Creates a Gradient Ascent 1D config
     *
     * @param f, std function pointer which returns a tuple with <F(X), F'(X)>
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
    Gradient1DConfig(std::function<std::tuple<float, float>(float)>* f,
                     float startX, float prevX, float dxError = 0.1,
                     float maxXMovement = 0.02, float temperatureDescent = 0.5,
                     float temperatureMin = 0.01, int maxIterations = 100,
                     float maxValue = 0, float maxThresh = 0)
        :

          f(f),
          startX(startX),
          prevX(prevX),
          dxError(dxError),
          maxXMovement(maxXMovement),
          temperatureDescent(temperatureDescent),
          temperatureMin(temperatureMin),
          maxIterations(maxIterations),
          maxValue(maxValue),
          maxThresh(maxThresh) {}

    std::function<std::tuple<float, float>(float)>* f;
    float startX;
    float prevX;
    float dxError;
    float maxXMovement;
    float temperatureDescent;
    float temperatureMin;
    int maxIterations;
    float maxValue;
    float maxThresh;
};