#include <gtest/gtest.h>
// #include <../../common/Point.hpp>
#include <Geometry2d/Point.hpp>

using namespace std;
using namespace Geometry2d;



TEST(testPoint, clampBig) {
    Point p0(10.0, 10.0);
    p0.x;
    p0.clamp(10);
    EXPECT_FLOAT_EQ(10, p0.mag());
}

//  if you clamp a vector to a value that's bigger
//  than it's magnitued, it shouldn't change
TEST(testPoint, clampSmall) {
    Point p0(10, 0);
    p0.clamp(15);
    EXPECT_FLOAT_EQ(10, p0.mag());
}
