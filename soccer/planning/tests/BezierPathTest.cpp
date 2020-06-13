#include <gtest/gtest.h>

#include <fstream>

#include "planning/low_level/PathSmoothing.hpp"

using Geometry2d::Point;
using Geometry2d::Point;

TEST(BezierPath, SmoothPath) {
    // Smoothed paths should have reasonably low curvature everywhere on the
    // path. Sadly, this does not hold with our current system, which uses
    // Bezier curves and places keypoints in a fairly naive manner.
    // TODO(Kyle): Switch to a scheme that minimizes sum of squared
    //  {acceleration/curvature/etc}. This should be fairly simple with Hermite
    //  splines, as acceleration on a point in a curve is a linear function of
    //  the endpoints' positions and velocities (so sub of squared acceleration
    //  is a quadratic in the velocities (decision variables))
    MotionConstraints constraints;
    std::vector<Point> points {Point{0, 0}, Point{1, 0}, Point{2, 0}};
    Planning::BezierPath path(std::move(points),
                              Point(0.01, 0),
                              Point(0.01, 0),
                              constraints);

    const int N = 1000;

    std::ofstream out("/tmp/bezier.csv");

    for (int i = 0; i <= N; i++) {
        double s = i / static_cast<double>(N);
        Point position;
        Point tangent;
        double curvature = 0;
        path.Evaluate(s, &position, &tangent, &curvature);

        out << s << ", "
            << position.x() << ", "
            << position.y() << ", "
            << tangent.x() << ", "
            << tangent.y() << ", "
            << curvature << std::endl;

        EXPECT_LT(curvature, 100);
    }
}