#include <fstream>

#include <gtest/gtest.h>

#include "planning/primitives/path_smoothing.hpp"

using rj_geometry::Point;

static void check_bezier_low_curvature(const planning::BezierPath& path) {
    // Expected error is O(1/N)
    constexpr int kN = 1000;
    double ds = 1.0 / static_cast<double>(kN);

    for (int i = 0; i <= kN; i++) {
        double s = i * ds;
        double curvature = 0;
        Point tangent;
        path.evaluate(s, nullptr, &tangent, &curvature);

        if (tangent.mag() > 1e-3) {
            EXPECT_LE(std::abs(curvature), 100);
        }
    }
}

static void check_bezier_smooth(const planning::BezierPath& path) {
    // Expected error decreases with high N
    constexpr int kN = 100;
    constexpr double kEpsilon = 1e-2;

    double ds = 1.0 / static_cast<double>(kN);

    for (int i = 0; i < kN; i++) {
        double s = i * ds;

        Point position;
        Point tangent;
        double curvature = 0;
        path.evaluate(s, &position, &tangent, &curvature);

        Point position_next;
        Point tangent_next;
        double curvature_next = 0;
        double h = 1e-6;
        path.evaluate(s + h, &position_next, &tangent_next, &curvature_next);

        EXPECT_LE(
            (0.5 * (tangent + tangent_next)).dist_to((position_next - position) / h),
            kEpsilon);

        double curvature_expected = tangent_next.angle_between(tangent) / h / tangent.mag();

        if (tangent.mag() > 1e-3 && tangent_next.mag() > 1e-3) {
            // Make sure that the approximate curvature is consistent with the
            // calculated exact value.
            EXPECT_NEAR(curvature, std::abs(curvature_expected), kEpsilon) << " at s = " << s;
        }
    }
}

TEST(BezierPath, two_points_path_smooth_and_consistent) {
    planning::MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{1, 1}};
    planning::BezierPath path(std::move(points), Point(1, 0), Point(1, 0), constraints);
    check_bezier_smooth(path);
}

TEST(BezierPath, multiple_points_path_smooth_and_consistent) {
    planning::MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{1, 1}, Point{2, 0}};
    planning::BezierPath path(std::move(points), Point(1, 0), Point(1, 0), constraints);
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

TEST(BezierPath, zero_velocity_endpoints_straight_smooth_and_consistent) {
    planning::MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{2, 0}};
    planning::BezierPath path(std::move(points), Point(0, 0), Point(0, 0), constraints);
    check_bezier_smooth(path);
}

TEST(BezierPath, zero_endpoints_curved_smooth_and_consistent) {
    planning::MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{1, 1}, Point{2, 0}};
    planning::BezierPath path(std::move(points), Point(0, 0), Point(0, 0), constraints);
    check_bezier_smooth(path);
    check_bezier_low_curvature(path);
}

TEST(BezierPath, nonzero_start_zero_end_curved_smooth_and_consistent) {
    planning::MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{2, 2}};
    planning::BezierPath path(std::move(points), Point(1, 0), Point(0, 0), constraints);
    check_bezier_smooth(path);
    check_bezier_low_curvature(path);
}
