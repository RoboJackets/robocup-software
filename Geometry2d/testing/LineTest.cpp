#include <gtest/gtest.h>

#include "Geometry2d/Circle.hpp"
#include "Geometry2d/Util.hpp"

using namespace Geometry2d;

TEST(Line, delta) {
    Line l({-1, -1}, {1, 1});

    EXPECT_NEAR(l.delta().x(), 2, 0.01);
    EXPECT_NEAR(l.delta().y(), 2, 0.01);
}

TEST(Line, equality) {
    Line l1({-1, -1}, {1, 1});
    Line l2({-1, -1}, {1, 1});
    Line l3({-2, -1}, {1, 1});

    EXPECT_TRUE(l1 == l2);
}

TEST(Line, transform) {
    Line l({-1, -1}, {1, 1});

    l.transform(TransformMatrix::kMirrorX);
    EXPECT_NEAR(l.pt[0].x(), 1, 0.01);
    EXPECT_NEAR(l.pt[0].y(), -1, 0.01);
    EXPECT_NEAR(l.pt[1].x(), -1, 0.01);
    EXPECT_NEAR(l.pt[1].y(), 1, 0.01);
}

TEST(Line, bad_line_intersects_circle) {
    Circle circle(Point(0, 0), 1);
    Line line(Point(-5, -1), Point(0, -1.001));

    EXPECT_FALSE(line.intersects(circle));
}

TEST(Line, intersects_circle) {
    Circle circle(Point(0, 0), 1);
    Line line(Point(-5, -1), Point(0, -1.001));

    Point p1;
    Point p2;
    bool intersects = line.intersects(circle, &p1, &p2);
    EXPECT_FALSE(intersects) << p1 << " " << p2;

    EXPECT_TRUE(Line(Point(-5, -1), Point(0, -1)).intersects(circle, &p1, &p2));
    EXPECT_TRUE(Point(0, -1).nearly_equals(p1));
    EXPECT_TRUE(Point(0, -1).nearly_equals(p2));

    EXPECT_TRUE(Line(Point(-5, 0), Point(-4, 0)).intersects(circle, &p1, &p2));

    EXPECT_TRUE(Point(1, 0).nearly_equals(p1)) << p1;
    EXPECT_TRUE(Point(-1, 0).nearly_equals(p2)) << p2;

    circle = Circle(Point(5.5, 10.5), 1.5);
    EXPECT_FALSE(Line(Point(7, 5), Point(7.01, 10.5)).intersects(circle, &p1, &p2));
    EXPECT_TRUE(Line(Point(7, 5), Point(7, 4)).intersects(circle, &p1, &p2));
    EXPECT_TRUE(Point(7, 10.5).nearly_equals(p1)) << p1;
    EXPECT_TRUE(Point(7, 10.5).nearly_equals(p2)) << p2;

    EXPECT_TRUE(Line(Point(5.5, 0), Point(5.5, 0.01)).intersects(circle, &p1, &p2));

    EXPECT_TRUE(Point(5.5, 12).nearly_equals(p1)) << p1;
    EXPECT_TRUE(Point(5.5, 9).nearly_equals(p2)) << p2;
}

TEST(Line, dist_to) {
    Line test(Point(5, -1), Point(10, 4));

    ASSERT_FLOAT_EQ(0, test.dist_to(Point(5, -1)));
    ASSERT_FLOAT_EQ(0, test.dist_to(Point(13, 7)));
    ASSERT_FLOAT_EQ(sqrt(2), test.dist_to(Point(12, 8)));
    ASSERT_FLOAT_EQ(sqrt(15 * 15 * 2), test.dist_to(Point(5 - 60, -1 - 60) + Point(-15, 15)));
}

TEST(Line, intersects) {
    Line test(Point(5, -1), Point(10, 4));
    Point temp(9, 2);
    ASSERT_FALSE(test.intersects(Line(Point(0, 0), Point(1, 1)), &temp));
    ASSERT_EQ(Point(9, 2), temp);

    ASSERT_TRUE(test.intersects(Line(Point(-14.5, 5), Point(-10, 4)), &temp));
    ASSERT_TRUE(Point(6.36364, 0.36364).nearly_equals(temp));
}

TEST(Line, nearest_point) {
    Line test(Point(-2.5, 4.5), Point(-4.5, 6.5));
    ASSERT_TRUE(Point(1, 1).nearly_equals(test.nearest_point(Point(1, 1))));
    ASSERT_TRUE(Point(1, 1).nearly_equals(test.nearest_point(Point(0, 0))));
}

TEST(Line, point_side) {
    Line l({-1, -1}, {1, 1});

    Point p1(0, 10);
    Point p2(0, -10);

    EXPECT_NEAR(l.point_side(p1), 20, 0.1);
    EXPECT_NEAR(l.point_side(p2), -20, 0.1);
}
