#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "Gradient1DConfig.hpp"
#include "GradientAscent1D.hpp"
#include "ParallelGradient1DConfig.hpp"
#include "ParallelGradientAscent1D.hpp"

using namespace std;

// Inverted porabola
static tuple<float, float> eval_function(float x) { return make_tuple(1 - x * x, -2 * x); }

// Tests general execution by placing one on each side of the parabola
TEST(ParallelGradientAscent1D, execute) {
    ParallelGradient1DConfig config;
    function<tuple<float, float>(float)> f = &eval_function;

    config.ga_config.emplace_back(&f, -1, -1.1, 0.01, 0.01, 0.5, 0.01, 100, 1, 0.001);
    config.ga_config.emplace_back(&f, 1, 1.1, 0.01, 0.01, 0.5, 0.01, 100, 1, 0.001);

    config.x_combine_thresh = 0.1;

    ParallelGradientAscent1D pga(&config);

    pga.execute();

    EXPECT_NEAR(pga.get_max_values().at(0), 1, 0.01);
    EXPECT_NEAR(pga.get_max_x_values().at(0), 0, 0.1);
    EXPECT_EQ(pga.get_max_values().size(), 1);  // Make sure they combined
}
