#pragma once

#include <Geometry2d/Point.hpp>

#include "planning/MotionConstraints.hpp"

namespace Planning {

/**
 * A spatial path with no time or angle information.
 */
class BezierPath {
public:
    /**
     * Construct a bezier path for a curve with a given velocity, going through
     * the specified points and with specified velocity at endpoints.
     *
     * @param points The points through which the final curve must pass. These
     *      will be used as the endpoints for consecutive Bezier segments. In
     *      total there will be points.size() - 1 curves. This must not be
     *      empty.
     * @param vi The initial velocity, used as a tangent for the first curve.
     * @param vf The final velocity, used as a tangent for the last curve.
     * @param motion_constraints Linear constraints for motion. These are used
     *      to approximate the time between control points, which is later used
     *      to match up tangent vectors (approximately) with velocities.
     */
    BezierPath(const std::vector<Geometry2d::Point>& points,
               Geometry2d::Point vi, Geometry2d::Point vf,
               MotionConstraints motion_constraints);

    /**
     * Evaluate the path at a particular instant drawn from [0, 1].
     * Calculates only those values requested (by passing in a non-null
     * pointer).
     *
     * @param s The index to sample along, in the range of [0, 1].
     * @param position Output parameter for the position at s, if non-null.
     * @param tangent Output parameter for the tangent vector at s, if non-null.
     * @param curvature Output parameter for the curvature at s, if non-null.
     */
    void Evaluate(double s, Geometry2d::Point* position = nullptr,
                  Geometry2d::Point* tangent = nullptr,
                  double* curvature = nullptr) const;

    [[nodiscard]] bool empty() const { return control.empty(); }
    [[nodiscard]] int size() const { return control.size(); }

    struct CubicBezierControlPoints {
        Geometry2d::Point p0, p1, p2, p3;

        CubicBezierControlPoints() = default;

        CubicBezierControlPoints(Geometry2d::Point p0, Geometry2d::Point p1,
                                 Geometry2d::Point p2, Geometry2d::Point p3)
            : p0(p0), p1(p1), p2(p2), p3(p3) {}
    };

private:
    std::vector<CubicBezierControlPoints> control;
};

}  // namespace Planning
