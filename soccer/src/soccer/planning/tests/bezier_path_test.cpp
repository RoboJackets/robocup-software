#include <fstream>

#include <gtest/gtest.h>

#include "planning/primitives/path_smoothing.hpp"

using rj_geometry::Point;

static void check_bezier_low_curvature(const Planning::BezierPath& path) {
    // Expected error is O(1/N)
    constexpr int kN = 1000;
    double ds = 1.0 / static_cast<double>(kN);

    for (int i = 0; i <= kN; i++) {
        double s = i * ds;
        double curvature = 0;
        path.evaluate(s, nullptr, nullptr, &curvature);

        EXPECT_LE(std::abs(curvature), 100);
    }
}

static void check_bezier_smooth(const Planning::BezierPath& path) {
    // Expected error decreases with high N
    constexpr int kN = 10000;
    constexpr double kEpsilon = 1e-2;

    Point previous_position;
    Point previous_velocity;
    path.evaluate(0, &previous_position, &previous_velocity, nullptr);

    double ds = 1.0 / static_cast<double>(kN);

    for (int i = 1; i <= kN; i++) {
        double s = i * ds;
        Point position;
        Point tangent;
        double curvature = 0;
        path.evaluate(s, &position, &tangent, &curvature);

        EXPECT_LE((0.5 * (previous_velocity + tangent) * ds).dist_to(position - previous_position),
                  kEpsilon);

        double curvature_expected =
            (tangent.normalized() - previous_velocity.normalized()).mag() / ds / tangent.mag();

        // Make sure that the approximate curvature is consistent with the
        // calculated exact value.
        EXPECT_NEAR(curvature, std::abs(curvature_expected), kEpsilon);

        previous_position = position;
        previous_velocity = tangent;
    }
}

TEST(BezierPath, two_points_path_smooth_and_consistent) {
    MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{1, 1}};
    Planning::BezierPath path(std::move(points), Point(1, 0), Point(1, 0), constraints);
    check_bezier_smooth(path);
}

TEST(BezierPath, multiple_points_path_smooth_and_consistent) {
    MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{1, 1}, Point{2, 0}};
    Planning::BezierPath path(std::move(points), Point(1, 0), Point(1, 0), constraints);
    check_bezier_smooth(path);
}

// Smoothed paths should have reasonably low curvature everywhere on the
// path. Sadly, this does not hold with our current system, which uses
// Bezier curves and places keypoints in a fairly naive manner.
// All examples with zero-velocity endpoints are broken because of numerical
// issues (in this case, the second and third control points go on top of the
// first and fourth, respectively).
// TODO(#1539): Switch to a scheme that minimizes sum of squared
//  {acceleration/curvature/etc}. This should be fairly simple with Hermite
//  splines, as acceleration on a point in a curve is a linear function of
//  the endpoints' positions and velocities (so sub of squared acceleration
//  is a quadratic in the velocities (decision variables))

TEST(BezierPath, DISABLED_zero_velocity_endpoints_straight_smooth_and_consistent) {
    MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{2, 0}};
    Planning::BezierPath path(std::move(points), Point(0, 0), Point(0, 0), constraints);
    check_bezier_smooth(path);
}

TEST(BezierPath, DISABLED_zero_endpoints_curved_smooth_and_consistent) {
    MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{1, 1}, Point{2, 0}};
    Planning::BezierPath path(std::move(points), Point(0, 0), Point(0, 0), constraints);
    check_bezier_smooth(path);
    check_bezier_low_curvature(path);
}

TEST(BezierPath, DISABLED_nonzero_start_zero_end_curved_smooth_and_consistent) {
    MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{2, 2}};
    Planning::BezierPath path(std::move(points), Point(1, 0), Point(0, 0), constraints);
    check_bezier_smooth(path);
    check_bezier_low_curvature(path);
}
