#include <gtest/gtest.h>
#include "GradientAscent1D.hpp"
#include "Gradient1DConfig.hpp"
#include "FunctionArgs.hpp"
#include <tuple>

using namespace std;

static tuple<float, float> evalFunction(float x, FunctionArgs* args) {
    return make_tuple(1 - x*x, -0.5 * x);
}

TEST(GradientAscent1D, execute) {
    FunctionArgs args;
    Gradient1DConfig config(&evalFunction, &args, -1, -1.1,
                            0.01, 0.01, 0.5, 0.01, 100, 1, 0.001);

    GradientAscent1D ga(&config);

    ga.execute();

    EXPECT_NEAR(ga.getValue(), 1, 0.01);
    EXPECT_NEAR(ga.getXValue(), 0, 0.1);
}
