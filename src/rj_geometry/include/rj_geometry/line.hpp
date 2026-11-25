#pragma once

#include <array>
#include <ostream>
#include <sstream>
#include <string>

#include <rj_geometry_msgs/msg/line.hpp>

#include "point.hpp"

namespace rj_geometry {

/**
 * An infinite line.
 * Parametrized by two distinct points on the line.
 */
class Line {
public:
    using Msg = rj_geometry_msgs::msg::Line;

    // --- Constructors ---

    Line() = default;
    /**
     * Initialize a line through two points.
     *
     * @param p1: a point on the line
     * @param p2: another point on the line
     *
     * NOTE: no assertions will be raised for degenerate lines (p1 == p2)
     */
    explicit Line(Point p1, Point p2) : points_{p1, p2} {}

    // --- Getters ---

    std::array<Point, 2> points() const { return points_; }
    Point delta() const { return points_[1] - points_[0]; }

    // --- Setters ---

    void set_points(Point p1, Point p2) { points_ = {p1, p2}; }

    // --- Operators ---

    bool operator==(const Line& other) const { return points_ == other.points_; }

    // --- Custom utilities ---

    /**
     * Returns the shortest distance from p to the line.
     */
    float dist_to(Point p) const;

    /**
     * Returns the point on the line closest to p.
     */
    Point nearest_point(Point p) const;

    /**
     * Returns which side of the line p is on, as the cross product of delta() and p - points()[0].
     * Positive is to the left (counterclockwise) when looking from points()[0] toward points()[1].
     */
    float point_side(const Point& p) const {
        Point d = delta();
        Point v = p - points_[0];

        return (d.x() * v.y() - v.x() * d.y());
    }

    std::string to_string() const {
        std::stringstream str;
        str << "Line<" << points_[0] << ", " << points_[1] << ">";
        return str.str();
    }
    friend std::ostream& operator<<(std::ostream& stream, const Line& line) {
        return stream << line.to_string();
    }

private:
    // --- Hidden parameters ---

    std::array<Point, 2> points_;
};

}  // namespace rj_geometry
