#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "Geometry2d/Arc.hpp"
#include "Geometry2d/Line.hpp"
#include "Geometry2d/Point.hpp"
#include "Geometry2d/Segment.hpp"

namespace Geometry2d::Testing {

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
    EXPECT_NEAR(a.radius_sq(), radius * radius, 0.001f);

    center = Point(2, 2);
    a.setCenter(center);
    radius = 1.2f;
    a.setRadius(radius);
    start = 0.3f;
    a.setStart(start);
    end = 0.4f;
    a.setEnd(end);

    EXPECT_NEAR(a.center().x(), center.x(), 0.001f);
    EXPECT_NEAR(a.center().y(), center.y(), 0.001f);
    EXPECT_NEAR(a.radius(), radius, 0.001f);
    EXPECT_NEAR(a.start(), start, 0.001f);
    EXPECT_NEAR(a.end(), end, 0.001f);
    EXPECT_NEAR(a.radius_sq(), radius * radius, 0.001f);
}

TEST(Arc, Intersections) {
    // Line-arc
    auto check_all_near = [](std::vector<Point> actual,
                             const std::vector<Point>& expected,
                             const std::string& name) {
        for (Point pt : expected) {
            auto it =
                std::find_if(actual.begin(), actual.end(),
                             [pt](Point ac) { return (pt - ac).mag() < 1e-6; });
            if (it != actual.end()) {
                actual.erase(it);
            } else {
                ADD_FAILURE() << "Expected point " << pt
                              << ", no match between " << name << ".";
            }
        }

        for (Point ac : actual) {
            ADD_FAILURE() << "Actual point " << ac
                          << " matched no expected point, "
                             "but all expected points were matched between "
                          << name << ".";
        }
    };

    // Arc-Line
    check_all_near(Arc({0, 0}, 1.0, 0, M_PI).intersects(Line({0, 0}, {0, 2})),
                   {{0, 1}}, "Arc-Line 1");
    check_all_near(
        Arc({0, 0}, 1.0, -M_PI, M_PI).intersects(Line({0, -2}, {0, 2})),
        {{0, 1}, {0, -1}}, "Arc-Line 2");
    check_all_near(Arc({0, 0}, 2.0, 0, M_PI).intersects(Line({0, 0}, {0, 1})),
                   {{0, 2}}, "Arc-Line 3");
    check_all_near(Arc({0, 0}, 1.0, M_PI / 4, 3 * M_PI / 4)
                       .intersects(Line({0, 0}, {0, 1})),
                   {{0, 1}}, "Arc-Line 4");

    // Arc-Line No intersection
    std::vector<Point> v =
        Arc({0, 0}, 2.0, 0, M_PI).intersects(Line({-1, -1}, {1, -1}));
    EXPECT_TRUE(v.empty());

    // Arc-Segment
    check_all_near(
        Arc({0, 0}, 2.0, 0, M_PI).intersects(Segment({0, 0}, {0, 1})), {},
        "Arc-Segment");
}

}  // namespace Geometry2d::Testing
