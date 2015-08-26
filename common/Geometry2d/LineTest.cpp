#include <gtest/gtest.h>
#include "Circle.hpp"

using namespace Geometry2d;

TEST(Line, intersectsCircle) {
    Circle circle(Point(0, 0), 1);
    Line line(Point(0, 0), Point(0, 0));

    Point p1, p2;
    bool intersects = line.intersects(circle, &p1, &p2);
    EXPECT_FALSE(intersects);
}
