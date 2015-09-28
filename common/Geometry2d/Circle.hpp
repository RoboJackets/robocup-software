#pragma once

#include "Shape.hpp"
#include "Point.hpp"
#include "Line.hpp"

namespace Geometry2d {

class Circle : public Shape {
public:
    Circle() {
        _r = -1;
        _rsq = -1;
    }

    Circle(Point c, float r) {
        center = c;
        _r = r;
        _rsq = -1;
    }

    Circle(const Circle& other) {
        center = other.center;
        _r = other.radius();
    }

    Shape* clone() const override;

    // Both radius and radius-squared are stored, since some operations are more
    // efficient with one or the other.
    // As long as one is specified, the other is calculated lazily.

    // Radius squared
    float radius_sq() const {
        if (_rsq < 0 && _r >= 0) {
            _rsq = _r * _r;
        }

        return _rsq;
    }

    void radius_sq(float value) {
        _rsq = value;
        _r = -1;
    }

    // Radius
    float radius() const {
        if (_r < 0 && _rsq >= 0) {
            _r = sqrtf(_rsq);
        }

        return _r;
    }

    void radius(float value) {
        _r = value;
        _rsq = -1;
    }

    bool containsPoint(Point pt) const override;

    bool hit(Point pt) const override;

    bool hit(const Segment& pt) const override;

    // Returns the number of points at which this circle intersects the given
    // circle.
    // i must be null or point to two points.
    // Only the first n points in i are modified, where n is the return value.
    int intersects(Circle& other, Point* i = nullptr) const;

    // Returns the number of points at which this circle intersects the given
    // line. i must be null or point to two points. Only the first n points in i
    // are modified, where n is the return value.
    int intersects(const Line& line, Point* i = nullptr) const;

    bool tangentPoints(Point src, Point* p1 = nullptr,
                       Point* p2 = nullptr) const;

    /// finds the point on the circle closest to @p
    Point nearestPoint(Point p) const;

    Point center;

    std::string toString() override {
        std::stringstream str;
        str << "Circle<" << center << ", " << radius() << ">";
        return str.str();
    }

protected:
    // Radius
    mutable float _r;

    // Radius squared
    mutable float _rsq;
};
}
