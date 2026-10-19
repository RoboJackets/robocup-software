#pragma once

#include <cassert>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <rj_geometry_msgs/msg/circle.hpp>
#include <rj_geometry_msgs/msg/detail/circle__struct.hpp>

#include "line.hpp"
#include "point.hpp"
#include "shape.hpp"

namespace rj_geometry {

/**
 * A circle :^)
 * Parametrized by a center point and a radius.
 */
class Circle : public Shape {
public:
    using Msg = rj_geometry_msgs::msg::Circle;

    /**
     * Initialize a circle with a given center and radius.
     *
     * @param center: the center point of the underlying circle
     * @param radius: the radius of the underlying circle in the range [0, infty)
     *
     * @note if radius < 0, an assertion will be thrown
     */
    Circle(Point center, float radius) {
        assert(0 <= radius);
        center_ = center;
        radius_ = radius;
    }

    /** Getters */
    Point center() const { return center_; }
    float radius() const { return radius_; }

    /** Setters */
    void set_center(Point center) {
        center_ = center;
    }
    void set_radius(float radius) {
        assert(0 <= radius);
        radius_ = radius;
    }

    /** Overriding virtuals from Shape */
    bool contains_point(const Point& p) const override;
    bool near_point(const Point& p, float clearance) const override;
    bool intersects_segment(const Segment& seg) const override;
    bool near_segment(const Segment& seg, float clearance) const override;
    std::vector<Point> get_segment_intersections(const Segment& seg) const override;
    std::string to_string() const override {
        std::stringstream str;
        str << "Circle<" << center_ << ", " << radius() << ">";
        return str.str();
    }

    /** Overriding clone utilites from Shape */
    std::unique_ptr<Circle> clone() const;

    /** Custom utilities not from Shape */
    // ... none for now ...

protected:
    /** Overriding clone utilites from Shape */
    Circle* clone_impl() const override;

    /** Hidden parameters */
    Point center_;
    float radius_;
};
}
