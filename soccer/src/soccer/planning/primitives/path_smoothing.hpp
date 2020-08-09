#pragma once

#include <rj_geometry/point.hpp>

#include "planning/motion_constraints.hpp"

namespace Planning {

/**
 * @brief A spatial path with no time or angle information.
 */
class BezierPath {
public:
    /**
     * @brief Construct a bezier path for a curve with a given velocity, going
     * through the specified points and with specified velocity at endpoints.
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
    BezierPath(const std::vector<rj_geometry::Point>& points,
               rj_geometry::Point vi, rj_geometry::Point vf,
               MotionConstraints motion_constraints);

    /**
     * @brief Evaluate the path at a particular instant drawn from [0, 1].
     * Calculates only those values requested (by passing in a non-null
     * pointer).
     *
     * @param s The index to sample along, in the range of [0, 1].
     * @param position Output parameter for the position at s, if non-null.
     * @param tangent Output parameter for the tangent vector at s, if non-null.
     * @param curvature Output parameter for the curvature at s, if non-null.
     */
    void evaluate(double s, rj_geometry::Point* position = nullptr,
                  rj_geometry::Point* tangent = nullptr,
                  double* curvature = nullptr) const;

    /**
     * @brief Whether or not this curve is empty.
     */
    [[nodiscard]] bool empty() const { return control_.empty(); }

    /**
     * @brief The number of cubic segments in this Bezier curve.
     */
    [[nodiscard]] int size() const { return control_.size(); }

    /**
     * @brief Control points for a Bezier curve.
     */
    struct CubicBezierControlPoints {
        rj_geometry::Point p0, p1, p2, p3;

        CubicBezierControlPoints() = default;

        CubicBezierControlPoints(rj_geometry::Point p0, rj_geometry::Point p1,
                                 rj_geometry::Point p2, rj_geometry::Point p3)
            : p0(p0), p1(p1), p2(p2), p3(p3) {}
    };

private:
    std::vector<CubicBezierControlPoints> control_;
};

}  // namespace Planning
