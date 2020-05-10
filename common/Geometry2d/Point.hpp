#pragma once

#include <protobuf/Point.pb.h>

#include <Eigen/Dense>
#include <QtCore/QPointF>
#include <boost/functional/hash.hpp>
#include <cmath>
#include <sstream>
#include <string>

namespace Geometry2d {
/**
Simple class to represent a point in 2d space. Uses floating point coordinates
*/
class Point {
public:
    [[nodiscard]] const double& x() const { return _x; }
    [[nodiscard]] const double& y() const { return _y; }
    double& x() { return _x; }
    double& y() { return _y; }

    /**
    sets the point to x,y
    @param x the x coordinate
    @param y the y coordinate
    */
    Point(double x = 0, double y = 0) : _x(x), _y(y) {}

    /**
     * Implicit constructor for creating a Point from a Packet::Point
     */
    Point(const Packet::Point& other) : Point(other.x(), other.y()) {}

    /**
     * Implicit constructor for creating a Point from a QPointF
     */
    Point(const QPointF& other) : Point(other.x(), other.y()) {}

    /**
     * Implicit constructor for creating a Point from a QPoint
     */
    Point(const QPoint& other) : Point(other.x(), other.y()) {}

    /**
     * Implicit constructor for creating a Point from a double*
     */
    Point(const double* other) : Point(other[0], other[1]) {}

    /**
     * Implicit conversion from Eigen::Vector2d
     */
    Point(const Eigen::Vector2d& other) : Point(other(0), other(1)) {}

    /**
     * Implicit conversion to Eigen::Vector2d
     */
    operator Eigen::Vector2d() const { return Eigen::Vector2d(x(), y()); }

    /**
     * to draw stuff and interface with QT
     */
    [[nodiscard]] QPointF toQPointF() const { return QPointF(x(), y()); }

    operator Packet::Point() const {
        Packet::Point out;
        out.set_x(static_cast<float>(x()));
        out.set_y(static_cast<float>(y()));
        return out;
    }

    /**
     * does vector addition
     * adds the + operator, shorthand
     */
    [[nodiscard]] Point operator+(Point other) const {
        return Point(x() + other.x(), y() + other.y());
    }

    /**
     * see operator+
     * does vector division, note the operator
     */
    [[nodiscard]] Point operator/(Point other) const {
        return Point(x() / other.x(), y() / other.y());
    }

    /**
     * @returns (x*x,y*y)
     */
    [[nodiscard]] Point operator*(Point other) const {
        return Point(x() * other.x(), y() * other.y());
    }

    /**
     * see operator+
     * does vector subtraction, note the operator
     * without parameter, it is the negative
     */
    [[nodiscard]] Point operator-(Point other) const {
        return Point(x() - other.x(), y() - other.y());
    }

    /**
     * multiplies the point by a -1 vector
     */
    [[nodiscard]] Point operator-() const { return Point(-x(), -y()); }

    /**
     * see operator+
     * this modifies the value instead of returning a new value
     */
    Point& operator+=(Point other) {
        x() += other.x();
        y() += other.y();

        return *this;
    }

    /**
     * see operator-
     * this modifies the value instead of returning a new value
     */
    Point& operator-=(Point other) {
        x() -= other.x();
        y() -= other.y();

        return *this;
    }

    /**
     * see operator*
     * this modifies the value instead of returning a new value
     */
    Point& operator*=(double s) {
        x() *= s;
        y() *= s;

        return *this;
    }

    /**
     * see operator/
     * this modifies the value instead of returning a new value
     */
    Point& operator/=(double s) {
        x() /= s;
        y() /= s;

        return *this;
    }

    /**
     * adds the / operator for vectors
     *  scalar division
     */
    [[nodiscard]] Point operator/(double s) const {
        return Point(x() / s, y() / s);
    }
    /**
     * adds the * operator for vectors
     * scalar multiplication
     */
    [[nodiscard]] Point operator*(double s) const {
        return Point(x() * s, y() * s);
    }

    /**
     * compares two points to see if both x and y are the same
     * adds the == operator
     */
    [[nodiscard]] bool operator==(Point other) const {
        return x() == other.x() && y() == other.y();
    }

    /**
     * this is the negation of operator operator !=
     */
    [[nodiscard]] bool operator!=(Point other) const {
        return x() != other.x() || y() != other.y();
    }

    [[nodiscard]] const double& operator[](int i) const {
        if (0 == i) {
            return _x;
        }
        if (1 == i) {
            return _y;
        }
        throw std::out_of_range("Out of range index for Geometry2d::Point");
    }

    double& operator[](int i) {
        if (0 == i) {
            return _x;
        }
        if (1 == i) {
            return _y;
        }
        throw std::out_of_range("Out of range index for Geometry2d::Point");
    }

    /**
     * Hash function for Geometry2d::Point
     */
    static size_t hash(Point pt) {
        size_t seed = 0;
        boost::hash_combine(seed, pt.x());
        boost::hash_combine(seed, pt.y());
        return seed;
    }

    /**
    computes the dot product of this point and another.
    behaves as if the points were 2d vectors
    @param p the second point
    @return the dot product of the two
    */
    [[nodiscard]] double dot(Point p) const {
        return x() * p.x() + y() * p.y();
    }

    /**
    computes the magnitude of the point, as if it were a vector
    @return the magnitude of the point
    */
    [[nodiscard]] double mag() const {
        return sqrtf(static_cast<float>(x() * x() + y() * y()));
    }

    /**
    computes magnitude squared
    this is faster than mag()
    @return the magnitude squared
    */
    [[nodiscard]] double magsq() const { return x() * x() + y() * y(); }

    /**
     * @brief Restricts the point to a given magnitude
     * @param max The magnitude to restrict the vector
     */
    Point& clamp(double max) {
        double ratio = mag() / max;
        if (ratio > 1) {
            x() /= ratio;
            y() /= ratio;
        }
        return *this;
    }

    /**
    rotates the point around another point by specified angle in the CCW
    direction
    @param origin the point to rotate around
    @param angle the angle in radians
    */
    Point& rotate(const Point& origin, double angle) {
        *this -= origin;
        rotate(angle);
        *this += origin;
        return *this;
    }

    /**
     * rotates the point around the origin
     */
    Point& rotate(double angle) {
        double newX = x() * cos(angle) - y() * sin(angle);
        double newY = y() * cos(angle) + x() * sin(angle);
        x() = newX;
        y() = newY;
        return *this;
    }

    /**
     * Like rotate(), but returns a new point instead of changing *this
     */
    [[nodiscard]] Point rotated(double angle) const {
        double newX = x() * cos(angle) - y() * sin(angle);
        double newY = y() * cos(angle) + x() * sin(angle);
        return Point(newX, newY);
    }

    /**
     * Returns a new Point rotated around the origin
     */
    [[nodiscard]] Point rotated(const Point& origin, double angle) const {
        return rotated(*this, origin, angle);
    }

    /**
     * static function to use rotate
     */
    static Point rotated(const Point& pt, const Point& origin, double angle) {
        Point newPt = pt;
        newPt.rotate(origin, angle);
        return newPt;
    }

    /**
    computes the distance from the current point to another
    @param other the point to find the distance to
    @return the distance between the points
    */
    [[nodiscard]] double distTo(const Point& other) const {
        Point delta = other - *this;
        return delta.mag();
    }

    /**
     * Returns a vector with the same direction as this vector but with
     * magnitude given, unless this vector is zero. If the vector is (0,0),
     * Point(0,0) is returned
     */
    [[nodiscard]] Point normalized(double magnitude = 1.0) const {
        double m = mag();
        if (m == 0) {
            return Point(0, 0);
        }

        return Point(magnitude * x() / m, magnitude * y() / m);
    }

    /// Alias for normalized() - matches Eigen's syntax
    [[nodiscard]] Point norm() const { return normalized(); }

    /**
     * Returns true if this point is within the given distance (threshold)
     * of (pt)
     */
    [[nodiscard]] bool nearPoint(const Point& other, double threshold) const {
        return (*this - other).magsq() <= (threshold * threshold);
    }

    /**
     * Returns the angle of this point in radians CCW from +X.
     */
    [[nodiscard]] double angle() const { return atan2(y(), x()); }

    /**
     * Returns a unit vector in the given direction (in radians)
     */
    static Point direction(double theta) {
        return Point(cos(theta), sin(theta));
    }

    /** returns the perpendicular to the point, Clockwise */
    [[nodiscard]] Point perpCW() const { return Point(y(), -x()); }

    /** returns the perpendicular to the point, Counter Clockwise */
    [[nodiscard]] Point perpCCW() const { return Point(-y(), x()); }

    /** saturates the magnitude of a vector */
    static Geometry2d::Point saturate(Geometry2d::Point value, double max) {
        double mag = value.mag();
        if (mag > fabs(max)) {
            return value.normalized() * fabs(max);
        }
        return value;
    }

    [[nodiscard]] double angleTo(const Point& other) const {
        return (other - *this).angle();
    }

    [[nodiscard]] double cross(const Point& other) const {
        return x() * other.y() - y() * other.x();
    }

    /** returns the angle between the two normalized points (radians) */
    [[nodiscard]] double angleBetween(const Point& other) const {
        double angle = normalized().dot(other.normalized());
        return acos(std::max(std::min(angle, 1.0), -1.0));
    }

    [[nodiscard]] bool nearlyEquals(Point other) const;

    [[nodiscard]] std::string toString() const {
        std::stringstream str;
        str << "Point(" << x() << ", " << y() << ")";
        return str.str();
    }

    friend std::ostream& operator<<(std::ostream& stream, const Point& point) {
        stream << point.toString();
        return stream;
    }

private:
    double _x, _y;
};  // \class Point

// global operations

/**
 * adds the * operator for vectors
 * scalar multiplication
 */
inline Point operator*(const double& s, const Point& pt) {
    return Point(pt.x() * s, pt.y() * s);
}
}  // namespace Geometry2d
