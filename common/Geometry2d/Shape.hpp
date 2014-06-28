#pragma once

#include "Point.hpp"


namespace Geometry2d {

    class Segment;

    class Shape {
    public:
        Shape() {}
        virtual ~Shape() {}

        bool containsPoint(const Point &pt) {
            return hit(pt);
        }

        /// collision-detection
        virtual bool hit(const Point &pt) const = 0;
        virtual bool hit(const Segment &seg) const = 0;
    };
}
