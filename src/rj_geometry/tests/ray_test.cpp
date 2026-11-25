#include <gtest/gtest.h>

#include "rj_geometry/point.hpp"
#include "rj_geometry/ray.hpp"

namespace rj_geometry::Testing {

TEST(Ray, GetterSetter) {
    Ray r(Point(1, 2), Point(0, 5));
    EXPECT_EQ(r.origin(), Point(1, 2));
    EXPECT_NEAR(r.direction().x(), 0, 1e-6);
    EXPECT_NEAR(r.direction().y(), 1, 1e-6);

    r.set_origin(Point(-1, -1));
    r.set_direction(Point(3, 0));
    EXPECT_EQ(r.origin(), Point(-1, -1));
    EXPECT_NEAR(r.direction().x(), 1, 1e-6);
    EXPECT_NEAR(r.direction().y(), 0, 1e-6);
}

TEST(Ray, NearestPoint) {
    Ray r(Point(0, 0), Point(1, 1));

    // Projects onto the ray in front of the origin
    Point n = r.nearest_point(Point(2, 0));
    EXPECT_NEAR(n.x(), 1, 1e-6);
    EXPECT_NEAR(n.y(), 1, 1e-6);

    // Far beyond the direction vector's length (no clamping at the far end)
    n = r.nearest_point(Point(100, 100));
    EXPECT_NEAR(n.x(), 100, 1e-6);
    EXPECT_NEAR(n.y(), 100, 1e-6);

    // Behind the origin clamps to the origin
    n = r.nearest_point(Point(-3, -1));
    EXPECT_NEAR(n.x(), 0, 1e-6);
    EXPECT_NEAR(n.y(), 0, 1e-6);
}

TEST(Ray, DistTo) {
    Ray r(Point(0, 0), Point(1, 0));
    EXPECT_NEAR(r.dist_to(Point(5, 3)), 3, 1e-6);
    EXPECT_NEAR(r.dist_to(Point(5, -3)), 3, 1e-6);
    EXPECT_NEAR(r.dist_to(Point(7, 0)), 0, 1e-6);
    EXPECT_NEAR(r.dist_to(Point(-3, 4)), 5, 1e-6);
}

TEST(Ray, Degenerate) {
    Ray r(Point(1, 1), Point(0, 0));
    EXPECT_EQ(r.nearest_point(Point(4, 5)), Point(1, 1));
    EXPECT_NEAR(r.dist_to(Point(4, 5)), 5, 1e-6);
}

}  // namespace rj_geometry::Testing
