#pragma once

#include "gradient_ascent1_d.hpp"
#include "parallel_gradient1_d_config.hpp"
#include <vector>

/**
 * Starts multiple "Gradient Ascent 1D" (GA1D) at various start points
 * Combines two single GA1D problems together when they are near the same X value
 *
 * Use Example:
 * PralellGradientAscent1D pga(& [ParallelGradient1DConfig obj]  );
 * pga.execute();
 * results = pgs.get_max_values();
 */
class ParallelGradientAscent1D {
public:
    ParallelGradientAscent1D(ParallelGradient1DConfig* config);

    /**
     * @brief Executes all problems until their max has been reached
     */
    void execute();

    /**
     * @return a list of all X values for each max in ascending order
     */
    std::vector<float> get_max_x_values();

    /**
     * @retunr a list of all the values for each max in ascending X order
     */
    std::vector<float> get_max_values();

private:
    ParallelGradient1DConfig* config_;

    std::vector<GradientAscent1D> problems_;
};