#include <gtest/gtest.h>

#include <rj_geometry/rect.hpp>
#include <rj_geometry/segment.hpp>
#include <rj_constants/constants.hpp>

namespace rj_geometry {

static Rect example(Point(0, 0), Point(2, 1));
static Rect intersect_example(Point(-1, 1), Point(1, 2));
static Rect null_example(Point(0, 0), Point(0, 0));

TEST(Rect, hit) {
    EXPECT_FALSE(example.hit(Point(10, 10)));
    EXPECT_TRUE(example.hit(Point(0, 0.5)));
    EXPECT_TRUE(example.hit(Point(0.5, example.maxy() + kRobotRadius - 0.01)));
    EXPECT_TRUE(example.hit(Point(0, 0)));
    EXPECT_TRUE(example.hit(Point(2, 1)));
    EXPECT_TRUE(example.hit(Point(0, 1)));
    EXPECT_TRUE(example.hit(Point(2, 0)));
    EXPECT_TRUE(example.hit(Point(1, -0.09)));
    EXPECT_FALSE(example.hit(Point(1, -0.091)));

    // degenerate case
    EXPECT_TRUE(null_example.hit(Point(0, 0)));
    EXPECT_TRUE(null_example.hit(Point(0, 0.09)));
    EXPECT_FALSE(null_example.hit(Point(0, 0.091)));
    EXPECT_FALSE(null_example.hit(Point(1, 1)));
}

TEST(Rect, contains_point) {
    EXPECT_TRUE(example.contains_point(Point(0, 0.5)));
    EXPECT_FALSE(example.contains_point(Point(-1, 01)));
}

TEST(Rect, cohen_codes){
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(0,1.2)), 0x00);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1,1)), 0x00);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1.5,1.5)), 0x01);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(1.5,1.2)), 0x02);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(1.5,2)), 0x02);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-.5,0)), 0x04);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1,0)), 0x04);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1,3)), 0x08);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-.5,3)), 0x08);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1.1,0)), 0x05);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(1.1,0)), 0x06);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1.5,2.5)), 0x09);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(1.5,2.5)), 0x0A);
}

TEST(Rect, degenerage_cohen_codes){
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(0,0)), 0x00);
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(1,0)), 0x02);
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(-1,0)), 0x01);
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(0,1)), 0x08);
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(0,-.1)), 0x04);
}

TEST(Rect, degenerage_segment_intersection) {
    std::tuple<bool, std::vector<Point> > res =
        null_example.intersects(Segment(Point(1, 0), Point(-1, 0)));
    EXPECT_TRUE(std::get<0>(res));
    res = null_example.intersects(Segment(Point(1, .1), Point(-1, 0)));
    EXPECT_FALSE(std::get<0>(res));
}

TEST(Rect, SegmentIntersection) {
    Point v1 = Point(-1, 1);
    Point v2 = Point(1, 2);
    std::tuple<bool, std::vector<Point> > res =
        intersect_example.intersects(Segment(Point(1, .1), Point(-1, 0)));
    EXPECT_FALSE(std::get<0>(res));
    res = intersect_example.intersects(Segment(Point(-2, .5), Point(2, 2.5)));
    std::vector<Point> intersection_points = std::get<1>(res);
    std::vector<rj_geometry::Point>::iterator it;
    EXPECT_TRUE(std::get<0>(res));
    EXPECT_TRUE(intersection_points.size() == 2);
    for (it = intersection_points.begin(); it != intersection_points.end(); it++) {
        EXPECT_TRUE(*it == v1 || *it == v2);
    }
}

}  // namespace rj_geometry
