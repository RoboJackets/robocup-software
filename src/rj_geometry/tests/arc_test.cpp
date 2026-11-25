#include <gtest/gtest.h>

#include "rj_geometry/arc.hpp"
#include "rj_geometry/point.hpp"

namespace rj_geometry::Testing {

TEST(Arc, GetterSetter) {
    Point center(1, 1);
    float radius = 1.1f;
    float start = 0.1f;
    float end = 0.2f;

    Arc a(center, radius, start, end);

    EXPECT_NEAR(a.center().x(), center.x(), 0.001f);
    EXPECT_NEAR(a.center().y(), center.y(), 0.001f);
    EXPECT_NEAR(a.radius(), radius, 0.001f);
    EXPECT_NEAR(a.start(), start, 0.001f);
    EXPECT_NEAR(a.end(), end, 0.001f);
    EXPECT_NEAR(a.radius() * a.radius(), radius * radius, 0.001f);

    center = Point(2, 2);
    a.set_center(center);
    radius = 1.2f;
    a.set_radius(radius);
    // Widen the end first so start <= end holds after each call
    end = 0.4f;
    a.set_end(end);
    start = 0.3f;
    a.set_start(start);

    EXPECT_NEAR(a.center().x(), center.x(), 0.001f);
    EXPECT_NEAR(a.center().y(), center.y(), 0.001f);
    EXPECT_NEAR(a.radius(), radius, 0.001f);
    EXPECT_NEAR(a.start(), start, 0.001f);
    EXPECT_NEAR(a.end(), end, 0.001f);
    EXPECT_NEAR(a.radius() * a.radius(), radius * radius, 0.001f);
}

}  // namespace rj_geometry::Testing
