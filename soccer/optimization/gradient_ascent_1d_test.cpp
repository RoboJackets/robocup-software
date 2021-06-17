#include <tuple>

#include <gtest/gtest.h>

#include "gradient_1d_config.hpp"
#include "gradient_ascent_1d.hpp"

using namespace std;

// Inverted parabola
static tuple<float, float> eval_function(float x) { return make_tuple(1 - x * x, -2 * x); }

// Tests general execution
TEST(GradientAscent1D, execute) {
    function<tuple<float, float>(float)> f = &eval_function;
    Gradient1DConfig config(&f, -1, -1.1, 0.01, 0.01, 0.5, 0.01, 1000, 1, 0.01);

    GradientAscent1D ga(&config);

    ga.execute();

    EXPECT_NEAR(ga.get_value(), 1, 0.01);
    EXPECT_NEAR(ga.get_x_value(), 0, 0.1);
}
