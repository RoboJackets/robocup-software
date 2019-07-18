#pragma once

#include "Point.hpp"

namespace Geometry2d {

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
    Eigen::Vector2d linear() const { return _twist.template block<2, 1>(0, 0); }
    double angular() const { return _twist(2); }

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
    Pose() = default;

    /**
     * Point-heading constructor
     */
    Pose(Point position, double heading)
        : _pose(position.x(), position.y(), heading) {}

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
        double x = other.position().x() + ch * other.position().x() -
                   sh * other.position().y();
        double y =
            other.position().y() + ch * position().y() + sh * position().x();
        double h = heading() + other.heading();
        return Pose({x, y}, h);
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
    Point position() const { return Point(_pose.template block<2, 1>(0, 0)); }
    double heading() const { return _pose(2); }

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
}  // namespace Geometry2d
