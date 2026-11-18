#include <vector>

#include <gtest/gtest.h>

#include "rj_geometry/circle.hpp"
#include "rj_geometry/point.hpp"
#include "rj_geometry/segment.hpp"

namespace rj_geometry::Testing {

TEST(Circle, GetterSetter) {
    Point center(1, 1);
    float radius = 1.1f;

    Circle c(center, radius);

    EXPECT_NEAR(c.center().x(), center.x(), 0.001);
    EXPECT_NEAR(c.center().y(), center.y(), 0.001);
    EXPECT_NEAR(c.radius(), radius, 0.001);

    center = Point(2, 3);
    c.set_center(center);
    radius = 2.2f;
    c.set_radius(radius);

    EXPECT_NEAR(c.center().x(), center.x(), 0.001);
    EXPECT_NEAR(c.center().y(), center.y(), 0.001);
    EXPECT_NEAR(c.radius(), radius, 0.001);
}

TEST(Circle, ContainsPt) {
    Circle c({0, 0}, 1);

    EXPECT_TRUE(c.contains_point({0.4, 0.4}));
    EXPECT_FALSE(c.contains_point({1.4, 0}));
}

TEST(Circle, Hit) {
    Circle c({0, 0}, 1);

    EXPECT_TRUE(c.hit(Point(0.4, 0.4)));
    EXPECT_FALSE(c.hit(Point(2, 0)));

    EXPECT_TRUE(c.hit(Segment({1, -1}, {1, 1})));
    EXPECT_FALSE(c.hit(Segment({4, -1}, {4, 1})));
}

TEST(Circle, NearPoint) {
    Circle c({0, 0}, 1);

    EXPECT_TRUE(c.near_point({0.5, 0}, 1));
    EXPECT_TRUE(c.near_point({1.5, 0}, 1));
    EXPECT_FALSE(c.near_point({2.5, 0}, 1));
}

TEST(Circle, SegmentIntersection) {
    Circle c({0, 0}, 1);

    EXPECT_TRUE(c.intersects_segment(Segment({-2, 0}, {2, 0})));
    EXPECT_TRUE(c.intersects_segment(Segment({0, 0}, {0.2, 0})));
    EXPECT_FALSE(c.intersects_segment(Segment({2, -1}, {2, 1})));

    const std::vector<Point> hits = c.get_segment_intersections(Segment({-2, 0}, {2, 0}));
    ASSERT_EQ(hits.size(), 2u);
    EXPECT_TRUE(hits[0].nearly_equals(Point(-1, 0)));
    EXPECT_TRUE(hits[1].nearly_equals(Point(1, 0)));

    const std::vector<Point> tangent = c.get_segment_intersections(Segment({1, -2}, {1, 2}));
    ASSERT_EQ(tangent.size(), 1u);
    EXPECT_TRUE(tangent[0].nearly_equals(Point(1, 0)));
}

}  // namespace rj_geometry::Testing
