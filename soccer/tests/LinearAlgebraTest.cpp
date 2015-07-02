#include <gtest/gtest.h>
#include <LinearAlgebra.hpp>

using namespace Eigen;

// Example unit test
TEST(LinearAlgebra, PseudoInverse) {
    Matrix2f orig;
    // clang-format off
    orig << 1, 2,
            3, 4;
    // clang-format on

    Matrix2f result = PseudoInverse(orig);

    // clang-format off
    Matrix2f expectedResult;
    expectedResult << -2,    1,
                     1.5, -0.5;
    // clang-format on
    EXPECT_FLOAT_EQ(expectedResult(0, 0), result(0, 0));
    EXPECT_FLOAT_EQ(expectedResult(0, 1), result(0, 1));
    EXPECT_FLOAT_EQ(expectedResult(1, 0), result(1, 0));
    EXPECT_FLOAT_EQ(expectedResult(1, 1), result(1, 1));
}
