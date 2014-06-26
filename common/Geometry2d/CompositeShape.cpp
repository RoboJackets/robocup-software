#include "CompositeShape.hpp"

using namespace Geometry2d;


bool Geometry2d::CompositeShape::containsPoint(const Point &pt) const {
    for (auto subshape : _subshapes) {
        if (subshape->containsPoint(pt)) return true;
    }
    return false;
}

void Geometry2d::CompositeShape::add(std::shared_ptr<Shape> shape) {
    if (shape) {
        _subshapes.push_back(shape);
    }
}

void Geometry2d::CompositeShape::add(const CompositeShape &compShape) {
    for (auto shape : compShape.subshapes()) {
        add(shape);
    }
}

void Geometry2d::CompositeShape::clear() {
    _subshapes.clear();
}
