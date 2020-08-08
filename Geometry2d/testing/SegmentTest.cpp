#include <gtest/gtest.h>

#include "Geometry2d/Segment.hpp"
#include "Geometry2d/Util.hpp"
using namespace Geometry2d;

TEST(Segment, dist_to) {
    Segment test(Point(5, -1), Point(10, 4));

    ASSERT_FLOAT_EQ(0, test.dist_to(Point(5, -1)));
    ASSERT_FLOAT_EQ(Point(10, 4).dist_to(Point(13, 7)), test.dist_to(Point(13, 7)));
    ASSERT_FLOAT_EQ(sqrt(2), test.dist_to(Point(7, 3)));
    Point test_point(Point(5, -1) + Point(15, 0));
    ASSERT_FLOAT_EQ(Point(10, 4).dist_to(test_point), test.dist_to(test_point));
}

TEST(Segment, intersects) {
    Segment test(Point(5, -1), Point(10, 4));
    Point temp(9, 2);
    ASSERT_FALSE(test.intersects(Segment(Point(0, 0), Point(1, 1)), &temp));
    ASSERT_EQ(Point(9, 2), temp);

    ASSERT_FALSE(test.intersects(Segment(Point(-14.5, 5), Point(-10, 4)), &temp));
    ASSERT_FALSE(test.intersects(Segment(Point(-14.5, 5), Point(6.3635, 0.36367)), &temp));
    ASSERT_TRUE(test.intersects(Segment(Point(-14.5, 5), Point(6.37, 0.362226)), &temp));
    ASSERT_TRUE(Point(6.36364, 0.36364).nearly_equals(temp));

    ASSERT_TRUE(test.intersects(Segment(Point(6.3635, 0.36367), Point(6.37, 0.362226)), &temp));
    ASSERT_TRUE(Point(6.36364, 0.36364).nearly_equals(temp));

    ASSERT_FALSE(test.intersects(Segment(Point(6.37, 0.362226), Point(1000, -220.444)), &temp));

    ASSERT_TRUE(Segment(Point(6.3635, 0.36367), Point(6.37, 0.362226)).intersects(test, &temp));
    ASSERT_TRUE(Point(6.36364, 0.36364).nearly_equals(temp));

    ASSERT_FALSE(Segment(Point(6.37, 0.362226), Point(1000, -220.444)).intersects(test, &temp));
}

TEST(Segment, nearest_point) {
    Segment test(Point(2.5, -0.5), Point(-4.5, 6.5));

    ASSERT_TRUE(Point(1, 1).nearly_equals(test.nearest_point(Point(1, 1))))
        << test.nearest_point(Point(1, 1));
    ASSERT_TRUE(Point(1, 1).nearly_equals(test.nearest_point(Point(0, 0))));
    test = Segment(Point(-2.5, 4.5), Point(-4.5, 6.5));

    ASSERT_TRUE(Point(-2.5, 4.5).nearly_equals(test.nearest_point(Point(1, 1))));
    ASSERT_TRUE(Point(-2.5, 4.5).nearly_equals(test.nearest_point(Point(0, 0))));

    test = Segment(Point(5, -1), Point(10, 4));

    ASSERT_TRUE(Point(5, -1).nearly_equals(test.nearest_point(Point(5, -1))));
    ASSERT_TRUE(Point(10, 4).nearly_equals(test.nearest_point(Point(13, 7))))
        << test.nearest_point(Point(13, 7));
}
