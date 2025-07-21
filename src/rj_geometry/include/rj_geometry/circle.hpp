#pragma once

#include <rj_geometry_msgs/msg/circle.hpp>

#include "line.hpp"
#include "point.hpp"
#include "shape.hpp"

namespace rj_geometry {

class Circle : public Shape {
public:
    using Msg = rj_geometry_msgs::msg::Circle;

    Circle() {
        r_ = 0;
        rsq_ = 0;
    }

    Circle(Point c, float r) {
        center = c;
        r_ = r;
        rsq_ = r_ * r_;
    }

    ~Circle() = default;
    Circle(const Circle& other) = default;
    Circle(Circle&& other) = default;
    Circle& operator=(const Circle& other) = default;
    Circle& operator=(Circle&& other) = default;

    Shape* clone() const override;

    // Both radius and radius-squared are stored, since some operations are more
    // efficient with one or the other.
    // As long as one is specified, the other is calculated lazily.

    // Radius squared
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
