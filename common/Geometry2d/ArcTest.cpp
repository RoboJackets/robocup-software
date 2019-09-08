#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "Geometry2d/Arc.hpp"
#include "Geometry2d/Line.hpp"
#include "Geometry2d/Point.hpp"
#include "Geometry2d/Segment.hpp"

namespace Geometry2d::Testing {

TEST(Arc, Intersections) {
    // Line-arc
    auto check_all_near = [](std::vector<Point> actual,
                             std::vector<Point> expected, std::string name) {
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

    // Arc-Segment
    check_all_near(
        Arc({0, 0}, 2.0, 0, M_PI).intersects(Segment({0, 0}, {0, 1})), {},
        "Arc-Segment");
}

}  // namespace Geometry2d::Testing
