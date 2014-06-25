#pragma once

#include "Point.hpp"


namespace Geometry2d {

    class Shape {
    public:
        Shape() {}
        virtual ~Shape() {}

        virtual bool containsPoint(const Point &pt) const;
    }
}
