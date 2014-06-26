#pragma once

#include "Point.hpp"


namespace Geometry2d {

    class Segment;

    class Shape {
    public:
        Shape() {}
        virtual ~Shape() {}

        // virtual bool containsPoint(const Point &pt) const = 0;

        /// collision-detection
        virtual bool hit(const Point &pt) const = 0;
        virtual bool hit(const Segment &seg) const = 0;
    };
}
