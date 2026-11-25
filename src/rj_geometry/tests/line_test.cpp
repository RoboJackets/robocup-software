#include "rj_geometry/line.hpp"

#include <gtest/gtest.h>

#include "rj_geometry/util.hpp"

using namespace rj_geometry;

TEST(Line, GetterSetter) {
    Line l({-1, -1}, {1, 1});
    EXPECT_EQ(l.points()[0], Point(-1, -1));
    EXPECT_EQ(l.points()[1], Point(1, 1));

    l.set_points({2, 3}, {4, 5});
    EXPECT_EQ(l.points()[0], Point(2, 3));
    EXPECT_EQ(l.points()[1], Point(4, 5));
}

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

TEST(Line, dist_to) {
    Line test(Point(5, -1), Point(10, 4));

    ASSERT_FLOAT_EQ(0, test.dist_to(Point(5, -1)));
    ASSERT_FLOAT_EQ(0, test.dist_to(Point(13, 7)));
    ASSERT_FLOAT_EQ(sqrt(2), test.dist_to(Point(12, 8)));
    ASSERT_FLOAT_EQ(sqrt(15 * 15 * 2), test.dist_to(Point(5 - 60, -1 - 60) + Point(-15, 15)));
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
