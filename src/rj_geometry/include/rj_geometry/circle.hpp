#pragma once

#include <cassert>
#include <rj_geometry_msgs/msg/circle.hpp>

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
    Circle(Point c, float r) {
        center = c;
        r_ = r;
        rsq_ = r_ * r_;
    }


    /** DEPRECATED 
     * This should be a copy constructor, no?
     */
    Shape* clone() const override;

    
    /** DEPRECATED
     * Why are we amortizing a multiplication bro
     */
    float radius_sq() const {
        return rsq_;
    }

    void radius_sq(float value) {
        rsq_ = value;
        r_ = sqrtf(rsq_);
    }

    // Radius
    float radius() const {
        return r_;
    }

    void radius(float value) {
        r_ = value;
        rsq_ = r_ * r_;
    }

    bool contains_point(Point pt) const override;

    bool hit(Point pt) const override;

    bool hit(const Segment& seg) const override;

    bool near_point(Point pt, float threshold) const override;

    // Returns the number of points at which this circle intersects the given
    // circle.
    // i must be null or point to two points.
    // Only the first n points in i are modified, where n is the return value.
    int intersects(Circle& other, Point* i = nullptr) const;

    // Returns the number of points at which this circle intersects the given
    // line. i must be null or point to two points. Only the first n points in i
    // are modified, where n is the return value.
    int intersects(const Line& line, Point* i = nullptr) const;

    bool tangent_points(Point src, Point* p1 = nullptr,
                       Point* p2 = nullptr) const;

    /// finds the point on the circle closest to @p
    Point nearest_point(Point p) const;

    Point center;
    

    std::string to_string() override {
        std::stringstream str;
        str << "Circle<" << center << ", " << radius() << ">";
        return str.str();
    }

protected:
    // Radius
    float r_;

    // Radius squared
    float rsq_;
};
}
