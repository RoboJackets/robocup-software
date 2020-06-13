#pragma once

#include <ostream>
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
    Pose() : _position{}, _heading{0} {}

    /**
     * Point-heading constructor
     */
    Pose(Point position, double heading)
        : _position(position), _heading(heading) {}

    /**
     * Component-wise constructor
     */
    Pose(double x, double y, double h) : _position(x, y), _heading(h) {}

    /**
     * Implicit conversion from Eigen::Vector3d
     */
    Pose(const Eigen::Vector3d& other)
        : _position(other(0), other(1)), _heading(other(2)) {}

    /**
     * Compute the pose specified using this pose as coordinates in a
     * frame of reference specified by `other`, in the global space.
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
    [[nodiscard]] Pose withOrigin(Pose other) const {
        Point rotated = position().rotated(other.heading());
        return other + Pose(rotated, heading());
    }

    /**
     * Implicit conversion to Eigen::Vector3d
     */
    [[nodiscard]] operator Eigen::Vector3d() const {
        return Eigen::Vector3d(position().x(), position().y(), heading());
    }

    /**
     * Calculate a TransformMatrix corresponding to using this pose as the
     * origin of the coordinate system.
     */
    [[nodiscard]] TransformMatrix transform() const {
        return TransformMatrix(position(), static_cast<float>(heading()));
    }

    /**
     * Accessors
     */
    Point& position() { return _position; }
    [[nodiscard]] Point const& position() const { return _position; }
    double& heading() { return _heading; }
    [[nodiscard]] double const& heading() const { return _heading; }

    /**
     * Operators
     */
    [[nodiscard]] Pose operator+(const Pose& other) const {
        return Pose(position() + other.position(), heading() + other.heading());
    }
    [[nodiscard]] Pose operator-(const Pose& other) const {
        return Pose(position() - other.position(), heading() - other.heading());
    }
    [[nodiscard]] Pose operator*(double s) const {
        return Pose(position() * s, heading() * s);
    }
    [[nodiscard]] Pose operator/(double s) const {
        return Pose(position() / s, heading() / s);
    }
    Pose& operator+=(const Pose& other) {
        position() += other.position();
        heading() += other.heading();
        return *this;
    }
    Pose& operator-=(const Pose& other) {
        position() += other.position();
        heading() += other.heading();
        return *this;
    }
    Pose& operator*=(double s) {
        position() *= s;
        heading() *= s;
        return *this;
    }
    Pose& operator/=(double s) {
        position() /= s;
        heading() /= s;
        return *this;
    }

    /**
     * Equality comparison operation.
     */
    bool operator==(const Pose& other) const {
        return _position == other._position && _heading == other._heading;
    }

    /**
     * Inequality comparison operation.
     */
    bool operator!=(const Pose& other) const {
        return _position != other._position || _heading != other._heading;
    }

    friend std::ostream& operator<<(std::ostream& stream, const Pose& pose) {
        stream << "Pose(" << pose.position().x() << ", " << pose.position().y()
               << ", " << pose.heading() << ")";
        return stream;
    }

    /**
     * Check whether two poses are "nearly equal", with RSSE
     * less than some tolerance (default: 1e-6).
     */
    static bool nearly_equals(const Pose& a, const Pose& b,
                              double tolerance = 1e-6) {
        double dx = a.position().x() - b.position().x();
        double dy = a.position().y() - b.position().y();
        double dh = a.heading() - b.heading();
        return dx * dx + dy * dy + dh * dh < tolerance * tolerance;
    }

private:
    Point _position;
    double _heading;
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
    Twist() : _linear{}, _angular{0} {}

    /**
     * Linear+angular terms.
     */
    Twist(const Point& linear, double angular)
        : _linear(linear), _angular(angular) {}

    /**
     * Component-wise constructor
     */
    Twist(double dx, double dy, double dh) : _linear(dx, dy), _angular(dh) {}

    /**
     * Implicit conversion from Eigen::Vector3d
     */
    Twist(const Eigen::Vector3d& other)
        : _linear(other(0), other(1)), _angular(other(2)) {}

    /**
     * Zero
     */
    static Twist Zero() { return Twist(Eigen::Vector3d::Zero()); }

    /**
     * Implicit conversion to Eigen::Vector3d
     */
    operator Eigen::Vector3d() const {
        return Eigen::Vector3d(linear().x(), linear().y(), angular());
    }

    /**
     * Accessors
     */
    Point& linear() { return _linear; }
    [[nodiscard]] Point const& linear() const { return _linear; }
    double& angular() { return _angular; }
    [[nodiscard]] double const& angular() const { return _angular; }

    /**
     * Find the resulting pose (delta) of an object starting at the origin
     * and continuing with constant (world-space) velocity for the specified
     * time (in seconds).
     *
     * Throughout the movement, linear velocity relative to the origin is
     * constant (but velocity in the pose's reference frame is changing in
     * direction as the pose rotates)
     *
     * Called deltaFixed because it operates fixed to the origin frame.
     */
    [[nodiscard]] Pose deltaFixed(double /*t*/) const {
        return Pose(linear().x(), linear().y(), angular());
    }

    /**
     * Find the resulting pose (delta) of an object starting at the origin and
     * continuing with constant (local) velocity for the specified time
     * (in seconds).
     *
     * Throughout the movement, linear velocity relative to the pose's frame of
     * reference is constant (but linear velocity relative to the origin might
     * change as the pose rotates)
     *
     * Called deltaRelative because it operates with velocities that remain
     * constant relative to the pose.
     *
     * In mathematical terms, this is the exponential mapping that takes the Lie
     * algebra se(2) (twists) to the Lie group SE(2) (poses).
     */
    [[nodiscard]] Pose deltaRelative(double t) const {
        // twist = (x', y', h')
        // dh(world) = h' * dt
        // dx(world) = dx(local)cos(dh(world)) - dy(local)sin(dh(world))
        //           = integral(x'cos(h't) - y'sin(h't), dt)
        //           = x'/h' sin(h't) + y'/h' (cos(h't) - 1)
        // dy(world) = dx(local)sin(dh(world)) + dy(local)cos(dh(world))
        //           = integral(x'sin(h't) + y'cos(h't), dt)
        //           = x'/h' (1 - cos(h't)) + y'/h' sin(h't)
        double vx = linear().x();
        double vy = linear().y();
        double vh = angular();

        // From above: sin(h't)/h' and (1 - cos(h't))/h' respectively
        double sine_frac;
        double cosine_frac;

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

        return Pose(vx * sine_frac - vy * cosine_frac,
                    vx * cosine_frac + vy * sine_frac, vh * t);
    }

    [[nodiscard]] double curvature() const {
        return angular() / linear().mag();
    }

    /**
     * Operators
     */
    [[nodiscard]] Twist operator+(const Twist& other) const {
        return Twist(linear() + other.linear(), angular() + other.angular());
    }
    [[nodiscard]] Twist operator-(const Twist& other) const {
        return Twist(linear() - other.linear(), angular() - other.angular());
    }
    [[nodiscard]] Twist operator*(double s) const {
        return Twist(linear() * s, angular() * s);
    }
    [[nodiscard]] Twist operator/(double s) const {
        return Twist(linear() / s, angular() / s);
    }
    Twist& operator+=(const Twist& other) {
        linear() += other.linear();
        angular() += other.angular();
        return *this;
    }
    Twist& operator-=(const Twist& other) {
        linear() -= other.linear();
        angular() -= other.angular();
        return *this;
    }
    Twist& operator*=(double s) {
        linear() *= s;
        angular() *= s;
        return *this;
    }
    Twist& operator/=(double s) {
        linear() /= s;
        angular() /= s;
        return *this;
    }

    /**
     * Equality comparison operation.
     */
    bool operator==(const Twist& other) const {
        return _linear == other._linear && _angular == other._angular;
    }

    /**
     * Inequality comparison operation.
     */
    bool operator!=(const Twist& other) const {
        return _linear != other._linear || _angular != other._angular;
    }

    friend std::ostream& operator<<(std::ostream& stream, const Twist& twist) {
        stream << "Twist(" << twist.linear().x() << ", " << twist.linear().y()
               << ", " << twist.angular() << ")";
        return stream;
    }

    /**
     * Check whether two twists are "nearly equal", with RSSE
     * less than some tolerance (default: 1e-6).
     */
    static bool nearly_equals(const Twist& a, const Twist& b,
                              double tolerance = 1e-6) {
        double dx = a.linear().x() - b.linear().x();
        double dy = a.linear().y() - b.linear().y();
        double dh = a.angular() - b.angular();
        return dx * dx + dy * dy + dh * dh < tolerance * tolerance;
    }

private:
    Point _linear;
    double _angular;
};

}  // namespace Geometry2d
