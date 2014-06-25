#pragma once

#include "Point.hpp"
#include <vector>


namespace Geometry2d {

    class CompositeShape {
    public:
        CompositeShape() {}
        virtual ~CompositeShape() {}

        virtual bool containsPoint(const Point &pt) const;

        void addSubshape(std::shared_ptr<Shape> shape);

        /// removes all subshapes
        void clear();


    private:
        std::vector<shared_ptr<Shape> > _subshapes;
    }
}
