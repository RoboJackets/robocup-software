#include "CompositeShape.hpp"

bool Geometry2d::CompositeShape::containsPoint(const Point &pt) {
    for (auto subshape in _subshapes) {
        if (subshape->containsPoint(pt)) return true;
    }
    return false;
}

void Geometry2d::CompositeShape::addSubshape(std::shared_ptr<Shape> shape) {
    if (shape) {
        _subshapes->push_back(shape);
    }
}

void Geometry2d::CompositeShape::clear() {
    _subshapes.clear();
}
