#include <gtest/gtest.h>

#include <rj_geometry/rect.hpp>
#include <rj_geometry/segment.hpp>
#include <rj_geometry/ssl_robot_constants.hpp>

namespace rj_geometry {

static Rect example(Point(0, 0), Point(2, 1));
static Rect intersect_example(Point(-1, 1), Point(1, 2));
static Rect null_example(Point(0, 0), Point(0, 0));

TEST(Rect, hit) {
    // hit() is the rectangle itself, including the boundary.
    EXPECT_FALSE(example.hit(Point(10, 10)));
    EXPECT_TRUE(example.hit(Point(0, 0.5)));
    EXPECT_TRUE(example.hit(Point(0, 0)));
    EXPECT_TRUE(example.hit(Point(2, 1)));
    EXPECT_TRUE(example.hit(Point(0, 1)));
    EXPECT_TRUE(example.hit(Point(2, 0)));
    EXPECT_FALSE(example.hit(Point(0.5, example.maxy() + kRobotRadius - 0.01)));
    EXPECT_FALSE(example.hit(Point(1, -0.09)));
    EXPECT_FALSE(example.hit(Point(1, -0.091)));

    // One robot of clearance is near_point, not hit().
    EXPECT_TRUE(example.near_point(Point(0.5, example.maxy() + kRobotRadius - 0.01), kRobotRadius));
    EXPECT_TRUE(example.near_point(Point(1, -0.09), kRobotRadius));
    EXPECT_FALSE(example.near_point(Point(1, -0.091), kRobotRadius));

    // degenerate case
    EXPECT_TRUE(null_example.hit(Point(0, 0)));
    EXPECT_FALSE(null_example.hit(Point(0, 0.09)));
    EXPECT_FALSE(null_example.hit(Point(0, 0.091)));
    EXPECT_FALSE(null_example.hit(Point(1, 1)));
    EXPECT_TRUE(null_example.near_point(Point(0, 0.09), kRobotRadius));
    EXPECT_FALSE(null_example.near_point(Point(0, 0.091), kRobotRadius));
}

TEST(Rect, contains_point) {
    EXPECT_TRUE(example.contains_point(Point(0, 0.5)));
    EXPECT_FALSE(example.contains_point(Point(-1, 01)));
}

TEST(Rect, intersects_segment) {
    // example is the axis-aligned rectangle with corners (0, 0) and (2, 1).

    // Misses the rectangle entirely.
    EXPECT_FALSE(example.intersects_segment(Segment(Point(3, 3), Point(4, 1))));
    EXPECT_FALSE(intersect_example.intersects_segment(Segment(Point(1, .1), Point(-1, 0))));

    // Crosses the interior, so both boundary crossings count.
    EXPECT_TRUE(example.intersects_segment(Segment(Point(-1, 0.5), Point(3, 0.5))));
    EXPECT_TRUE(intersect_example.intersects_segment(Segment(Point(-2, .5), Point(2, 2.5))));

    // Strictly inside. The filled rectangle overlaps the segment even though
    // the boundary walk reports no points.
    const Segment interior(Point(0.25, 0.25), Point(1.5, 0.5));
    EXPECT_TRUE(example.intersects_segment(interior));
    EXPECT_TRUE(example.get_segment_intersections(interior).empty());

    // One endpoint inside and one outside.
    EXPECT_TRUE(example.intersects_segment(Segment(Point(1, 0.5), Point(5, 0.5))));

    // Endpoint on the boundary, with the rest of the segment outside.
    EXPECT_TRUE(example.intersects_segment(Segment(Point(2, 0.5), Point(4, 0.5))));

    // Passes through a single corner and otherwise stays outside.
    EXPECT_TRUE(example.intersects_segment(Segment(Point(-1, 1), Point(1, -1))));

    // Overlaps an edge for a positive length.
    EXPECT_TRUE(example.intersects_segment(Segment(Point(-1, 0), Point(3, 0))));

    // A degenerate rectangle is the single point (0, 0).
    EXPECT_TRUE(null_example.intersects_segment(Segment(Point(1, 0), Point(-1, 0))));
    EXPECT_FALSE(null_example.intersects_segment(Segment(Point(1, .1), Point(-1, 0))));
}

TEST(Rect, SegmentIntersectionsOnBoundary) {
    EXPECT_FALSE(intersect_example.intersects_segment(Segment(Point(1, .1), Point(-1, 0))));
    EXPECT_TRUE(intersect_example.intersects_segment(Segment(Point(-2, .5), Point(2, 2.5))));

    std::vector<Point> points =
        intersect_example.get_segment_intersections(Segment(Point(-2, .5), Point(2, 2.5)));
    ASSERT_EQ(points.size(), 2u);
    EXPECT_TRUE(points[0] == Point(-1, 1) || points[0] == Point(1, 2));
    EXPECT_TRUE(points[1] == Point(-1, 1) || points[1] == Point(1, 2));

    // Collinear with the bottom edge: report both ends of the overlap.
    points = intersect_example.get_segment_intersections(Segment(Point(-2, 1), Point(2, 1)));
    ASSERT_EQ(points.size(), 2u);
    EXPECT_TRUE(points[0] == Point(-1, 1) || points[0] == Point(1, 1));
    EXPECT_TRUE(points[1] == Point(-1, 1) || points[1] == Point(1, 1));
}


}  // namespace rj_geometry
