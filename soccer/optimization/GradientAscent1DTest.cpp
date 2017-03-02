#include <gtest/gtest.h>
#include "GradientAscent1D.hpp"
#include "Gradient1DConfig.hpp"
#include <tuple>

using namespace std;

// Inverted parabola
static tuple<float, float> evalFunction(float x) {
    return make_tuple(1 - x * x, -2 * x);
}

// Tests general execution
TEST(GradientAscent1D, execute) {
    function<tuple<float, float>(float)> f = &evalFunction;
    Gradient1DConfig config(&f, -1, -1.1, 0.01, 0.01, 0.5, 0.01, 100, 1, 0.001);

    GradientAscent1D ga(&config);

    ga.execute();

    EXPECT_NEAR(ga.getValue(), 1, 0.01);
    EXPECT_NEAR(ga.getXValue(), 0, 0.1);
}
