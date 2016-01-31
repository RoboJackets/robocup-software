#include <gtest/gtest.h>
#include "Circle.hpp"
#include "Util.hpp"
using namespace Geometry2d;

TEST(Line, badLineIntersectsCircle) {
    Circle circle(Point(0, 0), 1);
    Line line(Point(-5, -1), Point(0, -1.001));

    EXPECT_FALSE(line.intersects(circle));
}

TEST(Line, intersectsCircle) {
    Circle circle(Point(0, 0), 1);
    Line line(Point(-5, -1), Point(0, -1.001));

    Point p1, p2;
    bool intersects = line.intersects(circle, &p1, &p2);
    EXPECT_FALSE(intersects) << p1 << " " << p2;

    EXPECT_TRUE(Line(Point(-5, -1), Point(0, -1)).intersects(circle, &p1, &p2));
    EXPECT_TRUE(Point(0, -1).nearlyEquals(p1));
    EXPECT_TRUE(Point(0, -1).nearlyEquals(p2));

    EXPECT_TRUE(Line(Point(-5,0), Point(-4,0)).intersects(circle, &p1, &p2));

    EXPECT_TRUE(Point(1, 0).nearlyEquals(p1)) << p1;
    EXPECT_TRUE(Point(-1, 0).nearlyEquals(p2)) << p2;

    circle = Circle(Point(5.5, 10.5), 1.5);
    EXPECT_FALSE(Line(Point(7, 5), Point(7.01, 10.5)).intersects(circle, &p1, &p2));
    EXPECT_TRUE(Line(Point(7, 5), Point(7, 4)).intersects(circle, &p1, &p2));
    EXPECT_TRUE(Point(7, 10.5).nearlyEquals(p1)) << p1;
    EXPECT_TRUE(Point(7, 10.5).nearlyEquals(p2)) << p2;

    EXPECT_TRUE(Line(Point(5.5,0), Point(5.5,0.01)).intersects(circle, &p1, &p2));

    EXPECT_TRUE(Point(5.5, 12).nearlyEquals(p1)) << p1;
    EXPECT_TRUE(Point(5.5, 9).nearlyEquals(p2)) << p2;
}

TEST(Line, distTo) {
    Line test(Point(5,-1), Point(10, 4));

    ASSERT_FLOAT_EQ(0, test.distTo(Point(5,-1)));
    ASSERT_FLOAT_EQ(0, test.distTo(Point(13,7)));
    ASSERT_FLOAT_EQ(sqrt(2), test.distTo(Point(12,8)));
    ASSERT_FLOAT_EQ(sqrt(15*15*2), test.distTo(Point(5-60,-1-60) + Point(-15,15)));
}

TEST(Line, intersects) {
    Line test(Point(5,-1), Point(10, 4));
    Point temp(9,2);
    ASSERT_FALSE(test.intersects(Line(Point(0,0), Point(1,1)), &temp));
    ASSERT_EQ(Point(9,2),temp);

    ASSERT_TRUE(test.intersects(Line(Point(-14.5,5), Point(-10,4)), &temp));
    ASSERT_TRUE(Point(6.36364, 0.36364).nearlyEquals(temp));
}

TEST(Line, nearestPoint) {
    Line test(Point(-2.5,4.5), Point(-4.5, 6.5));
    ASSERT_TRUE(Point(1,1).nearlyEquals(test.nearestPoint(Point(1,1))));
    ASSERT_TRUE(Point(1,1).nearlyEquals(test.nearestPoint(Point(0,0))));
}