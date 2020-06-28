#include "PathSmoothing.hpp"

#include <Utils.hpp>
#include <motion/TrapezoidalMotion.hpp>

namespace Planning {

using Geometry2d::Point;

/**
 * For each curve, calculate control points using a heuristic.
 *
 * @param vi The initial velocity of the robot on the path.
 * @param vf The final velocity of the robot on the path.
 * @param points The list of endpoints of the curves. The resulting curve will
 *      pass through these points. points should have length >= 2
 * @param ks An estimate of the time it will take to travel each curve.
 * @return A vector of control point locations.
 */
static void FitCubicBezier(
    Point vi, Point vf, const std::vector<Point>& points,
    const std::vector<double>& ks,
    std::vector<BezierPath::CubicBezierControlPoints>* control_out) {
    if (points.size() < 2) {
        throw std::invalid_argument(
            "Must have at least 2 points to fit cubic Bezier.");
    }

    int num_curves = static_cast<int>(points.size()) - 1;

    if (ks.size() != num_curves) {
        throw std::invalid_argument("Expected ks.size() == points.size() - 1");
    }

    control_out->resize(num_curves);
    for (int i = 0; i < num_curves; i++) {
        control_out->at(i).p0 = points[i];
        control_out->at(i).p3 = points[i + 1];
    }

    if (num_curves == 1) {
        // Special case for when we only have one curve. Then we put the first
        // (second) control point at the first (last) waypoint, extended by a
        // rescaled initial (final) velocity:
        // p0 ***** c0 --> v0
        //         ***
        //            ***
        //           c1  ****** p1 ---------> v1
        // (* represents path, pi represents points, ci represents control
        // points, vi represents velocity vectors)
        control_out->at(0).p1 = vi / (3 * ks[0]) + points[0];
        control_out->at(0).p2 = points[1] - vf / (3 * ks[0]);
        double assertTestValue =
            control_out->at(0).p1.mag() + control_out->at(0).p1.mag();
        if (!std::isfinite(assertTestValue)) {
            throw std::runtime_error(
                "Something went wrong. Points are too close to each other "
                "probably");
        }
    } else {
        using Eigen::MatrixXd;
        using Eigen::VectorXd;

        // Solve for the control points' coordinates as n equations in n
        // unknowns. We actually have x and y, but they turn out to be the same
        // equations (with different unknowns, so we can repeat the same
        // solution)
        int matrixSize = num_curves * 2;
        MatrixXd equations = MatrixXd::Zero(matrixSize, matrixSize);
        VectorXd answer_x(matrixSize);
        VectorXd answer_y(matrixSize);

        // These constraints come from the above special case. The first and
        // last control points should match up with that case.
        equations(0, 0) = 1;
        answer_x(0) = vi.x() / (3.0 * ks[0]) + points[0].x();
        answer_y(0) = vi.y() / (3.0 * ks[0]) + points[0].y();
        equations(1, matrixSize - 1) = 1;
        answer_x(1) =
            points[num_curves].x() - vf.x() / (3 * ks[num_curves - 1]);
        answer_y(1) =
            points[num_curves].y() - vf.y() / (3 * ks[num_curves - 1]);

        // Keep track of which equation we're on.
        int i = 2;

        // Equation i represents the constraint that for the border between
        // two given curves, their tangents should match (up to a rescaling by
        // ks)
        for (int n = 0; n < num_curves - 1; n++) {
            // ks[n]*(c1[n]-p1[n]) + ks[n+1]*(c0[n+1] - p0[n+1]) = 0
            // With p0[n+1] = p1[n].
            equations(i, n * 2 + 1) = ks[n];
            equations(i, n * 2 + 2) = ks[n + 1];
            answer_x(i) = (ks[n] + ks[n + 1]) * points[n + 1].x();
            answer_y(i) = (ks[n] + ks[n + 1]) * points[n + 1].y();
            i++;
        }

        // Finally, the last equations represent constraints between curves.
        // Specifically, we want the second derivatives ("acceleration") to
        // match up at the junctions between curves.
        for (int n = 0; n < num_curves - 1; n++) {
            // ks[n]^2 * c0[n] - 2 * ks[n]^2 * c1[n] + 2 * ks[n+1]^2 * c0[n+1] -
            // ks[n+1]^2 * c1[n+1]
            equations(i, n * 2) = ks[n] * ks[n];
            equations(i, n * 2 + 1) = -2 * ks[n] * ks[n];
            equations(i, n * 2 + 2) = 2 * ks[n + 1] * ks[n + 1];
            equations(i, n * 2 + 3) = -ks[n + 1] * ks[n + 1];
            answer_x(i) =
                points[n + 1].x() * (ks[n + 1] * ks[n + 1] - ks[n] * ks[n]);
            answer_y(i) =
                points[n + 1].y() * (ks[n + 1] * ks[n + 1] - ks[n] * ks[n]);
            i++;
        }

        Eigen::HouseholderQR<MatrixXd> solver(equations);
        VectorXd solution_x = solver.solve(answer_x);
        VectorXd solution_y = solver.solve(answer_y);

        if (solution_x.hasNaN() || solution_y.hasNaN()) {
            throw std::runtime_error(
                "Something went wrong. Points are too close to each other "
                "probably");
        }

        for (int n = 0; n < num_curves; n++) {
            control_out->at(n).p1 = Point(solution_x[n * 2], solution_y[n * 2]);
            control_out->at(n).p2 =
                Point(solution_x[n * 2 + 1], solution_y[n * 2 + 1]);
        }
    }
}

BezierPath::BezierPath(const std::vector<Point>& points, Point vi, Point vf,
                       MotionConstraints motion_constraints) {
    if (points.size() < 2) {
        return;
    }

    size_t length = points.size();
    int num_curves = static_cast<int>(length) - 1;

    // Each ks is the inverse of the time it takes to run that segment - it is
    // an approximation to s = ks*t for that segment (with s in [0, 1] and t in
    // [0, T])
    std::vector<double> ks(num_curves);

    const double startSpeed = vi.mag();

    const double endSpeed = vf.mag();

    // Approximate the curves as straight lines between the segments, and then
    // find an approximate ETA at each waypoint based on trapezoidal motion.

    double totalPathLength = 0.0;
    for (int i = 0; i < length - 1; i++) {
        totalPathLength += (points[i] - points[i + 1]).mag();
    }

    double pathLengthSoFar = 0.0;
    for (int i = 0; i < num_curves; i++) {
        double timeBefore = Trapezoidal::getTime(
            pathLengthSoFar, totalPathLength, motion_constraints.maxSpeed,
            motion_constraints.maxAcceleration, startSpeed, endSpeed);
        pathLengthSoFar += (points[i + 1] - points[i]).mag();
        double timeAfter = Trapezoidal::getTime(
            pathLengthSoFar, totalPathLength, motion_constraints.maxSpeed,
            motion_constraints.maxAcceleration, startSpeed, endSpeed);

        ks[i] = 1.0 / (timeAfter - timeBefore);

        if (!std::isfinite(ks[i])) {
            throw std::runtime_error(
                "Something went wrong. Points are too close to each other "
                "probably");
            return;
        }
    }

    // Finally, we can find our control points as the solutions to a set of
    // equations. This computation is carried out by cubicBezierCalc.
    FitCubicBezier(vi, vf, points, ks, &control);
}

void BezierPath::Evaluate(double s, Geometry2d::Point* position,
                          Geometry2d::Point* tangent, double* curvature) const {
    if (s < 0 || s > 1) {
        throw std::invalid_argument("Interpolant out of range.");
    }

    size_t num_curves = control.size();

    // First, find the curve to use.
    int index = static_cast<int>(s * num_curves);

    // This will only happen when s = 1 - in that case, we actually want to use
    // the last segment.
    if (index == num_curves) {
        index--;
    }

    // The remainder, from [0, 1] of how much of the curve at [index] is left.
    double t = s * num_curves - index;

    // Control points for this curve.
    Point p0 = control[index].p0;
    Point p1 = control[index].p1;
    Point p2 = control[index].p2;
    Point p3 = control[index].p3;

    // Equations from
    // https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B%C3%A9zier_curves.
    double tb = t;
    double te = 1 - t;
    using std::pow;
    Point pos = pow(te, 3) * p0 + 3 * te * te * tb * p1 +
                3 * te * tb * tb * p2 + pow(tb, 3) * p3;

    Point d1 = 3 * pow(te, 2) * (p1 - p0) + 6 * te * tb * (p2 - p1) +
               3 * tb * tb * (p3 - p2);

    if (position != nullptr) {
        *position = pos;
    }

    if (tangent != nullptr) {
        *tangent = d1;
    }

    if (curvature != nullptr) {
        Point d2 = (6 * te * (p2 - 2 * p1 + p0) + 6 * tb * (p3 - 2 * p2 + p1)) * num_curves;

        // https://en.wikipedia.org/wiki/Curvature#Local_expressions
        // K = |x'*y'' - y'*x''| / (x'^2 + y'^2)^(3/2) = |v x a|/|v|^3
        *curvature = std::abs(d1.cross(d2)) / std::pow(d1.mag(), 3.0);

        if (std::isnan(*curvature)) {
            *curvature = 0;
        }
    }
}

}  // namespace Planning
