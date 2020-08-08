#pragma once

#include "Shape.hpp"
#include "Point.hpp"
#include "Line.hpp"

namespace Geometry2d {

class Circle : public Shape {
public:
    Circle() {
        r_ = -1;
        rsq_ = -1;
    }

    Circle(Point c, float r) {
        center = c;
        r_ = r;
        rsq_ = -1;
    }

    Circle(const Circle& other) {
        center = other.center;
        r_ = other.radius();
    }

    Shape* clone() const override;

    // Both radius and radius-squared are stored, since some operations are more
    // efficient with one or the other.
    // As long as one is specified, the other is calculated lazily.

    // Radius squared
    float radius_sq() const {
        if (rsq_ < 0 && r_ >= 0) {
            rsq_ = r_ * r_;
        }

        return rsq_;
    }

    void radius_sq(float value) {
        rsq_ = value;
        r_ = -1;
    }

    // Radius
    float radius() const {
        if (r_ < 0 && rsq_ >= 0) {
            r_ = sqrtf(rsq_);
        }

        return r_;
    }

    void radius(float value) {
        r_ = value;
        rsq_ = -1;
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
    mutable float r_;

    // Radius squared
    mutable float rsq_;
};
}
