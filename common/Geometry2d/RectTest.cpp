#include <gtest/gtest.h>
#include <Geometry2d/Rect.hpp>
#include <Constants.hpp>

namespace Geometry2d {

static Rect example(Point(0, 0), Point(2, 1));

TEST(Rect, hit) {
    EXPECT_FALSE(example.hit(Point(10, 10)));
    EXPECT_TRUE(example.hit(Point(0, 0.5)));
    EXPECT_TRUE(example.hit(Point(0.5, example.maxy() + Robot_Radius - 0.01)));
    EXPECT_TRUE(example.hit(Point(0, 0)));
    EXPECT_TRUE(example.hit(Point(2, 1)));
    EXPECT_TRUE(example.hit(Point(0, 1)));
    EXPECT_TRUE(example.hit(Point(2, 0)));
    EXPECT_TRUE(example.hit(Point(1, -0.09)));
    EXPECT_FALSE(example.hit(Point(1, -0.091)));

    // degenerate case
    Rect example2(Point(0, 0), Point(0, 0));
    EXPECT_TRUE(example2.hit(Point(0, 0)));
    EXPECT_TRUE(example2.hit(Point(0, 0.09)));
    EXPECT_FALSE(example2.hit(Point(0, 0.091)));
    EXPECT_FALSE(example2.hit(Point(1, 1)));
}

TEST(Rect, containsPoint) {
    EXPECT_TRUE(example.containsPoint(Point(0, 0.5)));
    EXPECT_FALSE(example.containsPoint(Point(-1, 01)));
}

}  // namespace Geometry2d
