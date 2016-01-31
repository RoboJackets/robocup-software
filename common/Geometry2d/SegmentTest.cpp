#include <gtest/gtest.h>
#include "Segment.hpp"
#include "Util.hpp"
using namespace Geometry2d;




TEST(Segment, distTo) {
    Segment test(Point(5,-1), Point(10, 4));

    ASSERT_FLOAT_EQ(0, test.distTo(Point(5,-1)));
    ASSERT_FLOAT_EQ(Point(10,4).distTo(Point(13,7)), test.distTo(Point(13,7)));
    ASSERT_FLOAT_EQ(sqrt(2), test.distTo(Point(7,3)));
    Point testPoint(Point(5,-1) + Point(15,0));
    ASSERT_FLOAT_EQ(Point(10,4).distTo(testPoint), test.distTo(testPoint));
}


TEST(Segment, intersects) {
    Segment test(Point(5,-1), Point(10, 4));
    Point temp(9,2);
    ASSERT_FALSE(test.intersects(Segment(Point(0,0), Point(1,1)), &temp));
    ASSERT_EQ(Point(9,2),temp);

    ASSERT_FALSE(test.intersects(Segment(Point(-14.5,5), Point(-10,4)), &temp));
    ASSERT_FALSE(test.intersects(Segment(Point(-14.5,5), Point(6.3635,0.36367)), &temp));
    ASSERT_TRUE(test.intersects(Segment(Point(-14.5,5), Point(6.37,0.362226)), &temp));
    ASSERT_TRUE(Point(6.36364, 0.36364).nearlyEquals(temp));

    ASSERT_TRUE(test.intersects(Segment(Point(6.3635,0.36367), Point(6.37,0.362226)), &temp));
    ASSERT_TRUE(Point(6.36364, 0.36364).nearlyEquals(temp));

    ASSERT_FALSE(test.intersects(Segment(Point(6.37,0.362226), Point(1000, -220.444)), &temp));


    ASSERT_TRUE(Segment(Point(6.3635,0.36367), Point(6.37,0.362226)).intersects(test, &temp));
    ASSERT_TRUE(Point(6.36364, 0.36364).nearlyEquals(temp));

    ASSERT_FALSE(Segment(Point(6.37,0.362226), Point(1000, -220.444)).intersects(test, &temp));
}

TEST(Segment, nearestPoint) {
    Segment test(Point(2.5,-0.5), Point(-4.5, 6.5));

    ASSERT_TRUE(Point(1,1).nearlyEquals(test.nearestPoint(Point(1,1)))) << test.nearestPoint(Point(1,1));
    ASSERT_TRUE(Point(1,1).nearlyEquals(test.nearestPoint(Point(0,0))));
    test = Segment(Point(-2.5,4.5), Point(-4.5, 6.5));

    ASSERT_TRUE(Point(-2.5,4.5).nearlyEquals(test.nearestPoint(Point(1,1))));
    ASSERT_TRUE(Point(-2.5,4.5).nearlyEquals(test.nearestPoint(Point(0,0))));

    test = Segment(Point(5,-1), Point(10, 4));

    ASSERT_TRUE(Point(5,-1).nearlyEquals(test.nearestPoint(Point(5,-1))));
    ASSERT_TRUE(Point(10,4).nearlyEquals(test.nearestPoint(Point(13,7)))) << test.nearestPoint(Point(13,7));
}
