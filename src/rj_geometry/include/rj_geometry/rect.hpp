#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <memory>
#include <ostream>
#include <string>
#include <sstream>
#include <vector>
#include <rj_geometry_msgs/msg/rect.hpp>
#include <rj_geometry_msgs/msg/detail/rect__struct.hpp>

#include "point.hpp"
#include "shape.hpp"

namespace rj_geometry {
class Segment;

/**
 * A rectangle :D
 * Parametrized by two diagonally opposite corners.
 * They may be any diagonally opposite pair, it's internally canonicalized.
 */
class Rect : public Shape {
public:
    using Msg = rj_geometry_msgs::msg::Rect;

    // --- Constructors

    /** Degerate (single point rect) constructor */
    Rect() {}
    /** Degenerate (single line rect) constructor */
    Rect(Point p1) { corners_[0] = corners_[1] = p1; }
    /**
     * Initialize a rectangle given diagonally opposite corners.
     *
     * @param p1: any corner
     * @param p2: the corner diagonally opposite p1
     *
     * NOTE: no assertions will be raised for degenerate rectangles
     */
    Rect(Point p1, Point p2) {
        corners_[0] = p1;
        corners_[1] = p2;
    }

    // --- Getters ---

    std::array<Point, 2> corners() const { return corners_; };
    float minx() const { return std::min(corners_[0].x(), corners_[1].x()); }
    float miny() const { return std::min(corners_[0].y(), corners_[1].y()); }
    float maxx() const { return std::max(corners_[0].x(), corners_[1].x()); }
    float maxy() const { return std::max(corners_[0].y(), corners_[1].y()); }
    /**
     * This returns the 4 corners of the rectangle in a predictable order.
     * Works regardless of the corner pair specified in the construction.
     * <BottomLeft, TopLeft, TopRight, BottomRight>
     */
    std::array<Point, 4> all_corners() const {
        std::array<Point, 4> tmp;
        tmp[0] = Point(minx(), miny()); // BottomLeft
        tmp[1] = Point(minx(), maxy()); // TopLeft
        tmp[2] = Point(maxx(), maxy()); // TopRight
        tmp[3] = Point(maxx(), miny()); // BottomRight
        return tmp;
    }
    Point center() const { return (corners_[0] + corners_[1]) / 2; }

    // --- Setters ---

    void set_corners(Point p1, Point p2) {
        corners_[0] = p1;
        corners_[1] = p2;
    }
    /**
     * This expands the rectangle such that it includes the the given point.
     * 
     * NOTE: This will override the backing corner pair to the canon order:
     * [BottomLeft, TopRight]
     */
    void expand(Point pt);
    /**
     * This expands the rectangle such that it includes the given rect.
     *
     * NOTE: This will override the backing corner pair to the canon order:
     * [BottomLeft, TopRight]
     */
    void expand(const Rect& rect);
    /**
     * This enlarges the rectangle outward in all directions,
     * moving the edges by the specified padding amount.
     *
     * Note that this is L∞ padding not L2 padding, i.e.,
     * the result will still be a Rect, not some rounded corners.
     *
     * NOTE: This will override the backing corner pair to the canon order:
     * [BottomLeft, TopRight]
     */
    void pad(float padding);

    // --- Operators ---

    bool operator==(const Rect& other) const { return corners_ == other.corners_; }
    Rect operator+(Point offset) {
        return Rect(corners_[0] + offset, corners_[1] + offset);
    }
    Rect& operator+=(Point offset) {
        corners_[0] += offset;
        corners_[1] += offset;
        return *this;
    }
    Rect operator-(Point offset) {
        return Rect(corners_[0] - offset, corners_[1] - offset);
    }
    Rect& operator-=(Point offset) {
        corners_[0] -= offset;
        corners_[1] -= offset;
        return *this;
    }
    Rect operator*(float s) {
        return Rect(corners_[0] * s, corners_[1] * s);
    }
    Rect& operator*=(float s) {
        corners_[0] *= s;
        corners_[1] *= s;
        return *this;
    }
    Rect operator/(float s) {
        return Rect(corners_[0] / s, corners_[1] / s);
    }
    Rect& operator/=(float s) {
        corners_[0] /= s;
        corners_[1] /= s;
        return *this;
    }

    // --- Shape virtuals ---

    bool contains_point(const Point& p) const override;
    bool intersects_segment(const Segment& seg) const override;
    bool near_point(const Point& p, float clearance) const override;
    bool near_segment(const Segment& seg, float clearance) const override;
    std::vector<Point> get_segment_intersections(const Segment& seg) const override;
    std::string to_string() const override {
        std::stringstream str;
        std::array<Point, 4> c = all_corners();
        str << "Rect<" << c[0] << "," << c[2] << ">";
        return str.str();
    }

    // --- Shape pseudovirtual clone ---

    std::unique_ptr<Rect> clone() const;

protected:
    // --- Hidden parameters ---

    std::array<Point, 2> corners_;

    // --- Shape virtuals ---

    Rect* clone_impl() const override;
};
}  // namespace rj_geometry
