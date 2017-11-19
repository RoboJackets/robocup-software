#pragma once

#include "GradientAscent1D.hpp"
#include "ParallelGradient1DConfig.hpp"
#include <vector>

/**
 * Starts multiple "Gradient Ascent 1D" (GA1D) at various start points
 * Combines two single GA1D's together when they are near the same X value
 *
 * Use Example:
 * PralellGradientAscent1D pga(& [ParallelGradient1DConfig obj]  );
 * pga.execute();
 * results = pgs.getMaxValues();
 */
class ParallelGradientAscent1D {
public:
    ParallelGradientAscent1D(ParallelGradient1DConfig* config);

    /**
     * @brief Executes all GA1Ds until their max has been reached
     */
    void execute();

    /**
     * @return a list of all X values for each max in ascending order
     */
    std::vector<float> getMaxXValues();

    /**
     * @retunr a list of all the values for each max in ascending X order
     */
    std::vector<float> getMaxValues();

private:
    ParallelGradient1DConfig* config;

    std::vector<GradientAscent1D> GA1Ds;
};