#pragma once

#include "Point.hpp"
#include "TransformMatrix.hpp"

namespace Geometry2d {

/**
 * Represents a pose in 2d space: (x, y, theta). Uses double-precision floating
 * point numbers for the coordinates, and can be used to represent the position
 * and orientation of a robot in the plane.
 */
class Pose {
public:
    /**
     * Default constructor - zero-initialize
     */
    Pose() { _pose = Eigen::Vector3d::Zero(); }

    /**
     * Point-heading constructor
     */
    Pose(Point position, double heading)
        : _pose(position.x(), position.y(), heading) {}

    /**
     * Component-wise constructor
     */
    Pose(double x, double y, double h) : _pose(x, y, h) {}

    /**
     * Implicit conversion from Eigen::Vector3d
     */
    Pose(const Eigen::Vector3d& other) : _pose(other) {}

    /**
     * Copy-constructor - default
     */
    Pose(const Pose& other) = default;

    /**
     * Compute the pose specified using this pose as coordinates in a frame of
     * reference specified by `other`, in the global space.
     *
     * i.e.
     *
     * x:
     *       |  x
     *       | /
     *       |/
     * ------|-------
     *       |
     *       |
     *       |
     *
     * y:
     *     |
     *     |
     * ----|----
     *     |\
     *     | y
     *
     * y.withOrigin(x):
     *       |  x
     *       | / \
     *       |/   y
     * ------|------
     *       |
     */
    Pose withOrigin(Pose other) const {
        double sh = std::sin(other.heading()), ch = std::cos(other.heading());
        Point rotated = position().rotated(other.heading());
        return other + Pose(rotated, heading());
    }

    /**
     * Implicit conversion to Eigen::Vector3d
     */
    operator Eigen::Vector3d() const { return _pose; }

    /**
     * Calculate a TransformMatrix corresponding to using this pose as the
     * origin of the coordinate system.
     */
    TransformMatrix transform() const {
        return TransformMatrix(position(), heading());
    }

    /**
     * Accessors
     */
    Point const position() const {
        return Point(_pose.template block<2, 1>(0, 0));
    }
    double const heading() const { return _pose(2); }

    /**
     * Operators
     */
    Pose operator+(const Pose& other) const {
        return Pose(_pose + other._pose);
    }
    Pose operator-(const Pose& other) const {
        return Pose(_pose - other._pose);
    }
    Pose operator*(double s) const { return Pose(_pose * s); }
    Pose operator/(double s) const { return Pose(_pose / s); }
    Pose& operator+=(const Pose& other) {
        _pose += other._pose;
        return *this;
    }
    Pose& operator-=(const Pose& other) {
        _pose -= other._pose;
        return *this;
    }
    Pose& operator*=(double s) {
        _pose *= s;
        return *this;
    }
    Pose& operator/=(double s) {
        _pose /= s;
        return *this;
    }

private:
    Eigen::Vector3d _pose;
};

/**
 * Represents a differential (velocity, acceleration, etc.) in 2d space:
 * (dx, dy, dh). Uses double-precision floating point numbers.
 */
class Twist {
public:
    /**
     * Default constructor - zero-initialize
     */
    Twist() = default;

    /**
     * Linear+angular terms.
     */
    Twist(const Point& linear, double angular)
        : _twist(linear.x(), linear.y(), angular) {}

    /**
     * Component-wise constructor
     */
    Twist(double dx, double dy, double dh) : _twist(dx, dy, dh) {}

    /**
     * Implicit conversion from Eigen::Vector3d
     */
    Twist(const Eigen::Vector3d& other) : _twist(other) {}

    /**
     * Copy-constructor - default
     */
    Twist(const Twist& other) = default;

    /**
     * Zero
     */
    static Twist Zero() { return Twist(Eigen::Vector3d::Zero()); }

    /**
     * Implicit conversion to Eigen::Vector3d
     */
    operator Eigen::Vector3d() const { return _twist; }

    /**
     * Accessors
     */
    Point const linear() const {
        return Point(_twist.template block<2, 1>(0, 0));
    }
    double const angular() const { return _twist(2); }

    /**
     * Find the resulting pose (delta) of an object starting at the origin and
     * continuing with constant (world-space) velocity for the specified time
     * (in seconds).
     *
     * Throughout the movement, linear velocity relative to the origin is
     * constant (but velocity in the pose's reference frame is changing in
     * direction as the pose rotates)
     *
     * Called deltaFixed because it operates fixed to the origin frame.
     */
    Pose applyFixed(double t) const { return Pose(t * _twist); }

    /**
     * Find the resulting pose (delta) of an object starting at the origin and
     * continuing with constant (local) velocity for the specified time
     * (in seconds).
     *
     * Throughout the movement, linear velocity relative to the pose's frame of
     * reference is constant (but linear velocity relative to the origin might
     * change as the pose rotates)
     *
     * Called applyRelative because it operates with velocities that remain
     * constant relative to the pose.
     *
     * In mathematical terms, this is the exponential mapping that takes the Lie
     * algebra so(2) (twists) to the Lie group SO(2) (poses).
     */
    Pose applyRelative(double t) const {
        // twist = (x', y', h')
        // dh(world) = h' * dt
        // dx(world) = dx(local)cos(dh(world)) - dy(local)sin(dh(world))
        //           = integral(x'cos(h't) - y'sin(h't), dt)
        //           = x'/h' sin(h't) + y'/h' (cos(h't) - 1)
        // dy(world) = dx(local)sin(dh(world)) + dy(local)cos(dh(world))
        //           = integral(x'sin(h't) + y'cos(h't), dt)
        //           = x'/h' (1 - cos(h't)) + y'/h' sin(h't)
        double vx = _twist(0);
        double vy = _twist(1);
        double vh = _twist(2);

        // From above: sin(h't)/h' and (1 - cos(h't))/h' respectively
        double sine_frac, cosine_frac;

        if (std::abs(vh) < 1e-6) {
            // Small-angle approximations
            // sin(h't) ~ h't
            // 1 - cos(h't) ~ 0
            sine_frac = t;
            cosine_frac = 0;
        } else {
            sine_frac = std::sin(vh * t) / vh;
            cosine_frac = (1 - std::cos(vh * t)) / vh;
        }

        return Eigen::Vector3d{vx * sine_frac - vy * cosine_frac,
                               vx * cosine_frac + vy * sine_frac, vh * t};
    }

    double curvature() const { return angular() / linear().mag(); }

    /**
     * Operators
     */
    Twist operator+(const Twist& other) const {
        return Twist(_twist + other._twist);
    }
    Twist operator-(const Twist& other) const {
        return Twist(_twist - other._twist);
    }
    Twist operator*(double s) const { return Twist(_twist * s); }
    Twist operator/(double s) const { return Twist(_twist / s); }
    Twist& operator+=(const Twist& other) {
        _twist += other._twist;
        return *this;
    }
    Twist& operator-=(const Twist& other) {
        _twist -= other._twist;
        return *this;
    }
    Twist& operator*=(double s) {
        _twist *= s;
        return *this;
    }
    Twist& operator/=(double s) {
        _twist /= s;
        return *this;
    }

private:
    Eigen::Vector3d _twist;
};

}  // namespace Geometry2d
