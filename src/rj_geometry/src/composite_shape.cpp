#include <rj_geometry/composite_shape.hpp>

namespace rj_geometry {

std::unique_ptr<CompositeShape> CompositeShape::clone() const {
    return std::unique_ptr<CompositeShape>(clone_impl());
}

CompositeShape* CompositeShape::clone_impl() const { return new CompositeShape(*this); }

bool CompositeShape::contains_point(const Point& pt) const {
    // True if any subshape contains the point.
    for (const auto& subshape : subshapes_) {
        if (subshape->contains_point(pt)) {
            return true;
        }
    }
    return false;
}

bool CompositeShape::near_point(const Point& pt, float threshold) const {
    // True if any subshape reports the point as near.
    for (const auto& subshape : subshapes_) {
        if (subshape->near_point(pt, threshold)) {
            return true;
        }
    }
    return false;
}

bool CompositeShape::intersects_segment(const Segment& seg) const {
    // True if any subshape meets the segment.
    for (const auto& subshape : subshapes_) {
        if (subshape->intersects_segment(seg)) {
            return true;
        }
    }
    return false;
}

bool CompositeShape::near_segment(const Segment& seg, float clearance) const {
    // True if any subshape reports the segment as near.
    for (const auto& subshape : subshapes_) {
        if (subshape->near_segment(seg, clearance)) {
            return true;
        }
    }
    return false;
}

std::vector<Point> CompositeShape::get_segment_intersections(const Segment& seg) const {
    std::vector<Point> points;
    // Append every subshape's hits. A point on a shared boundary may appear more than once.
    for (const auto& subshape : subshapes_) {
        const std::vector<Point> hits = subshape->get_segment_intersections(seg);
        points.insert(points.end(), hits.begin(), hits.end());
    }
    return points;
}

void CompositeShape::add(const std::shared_ptr<Shape>& shape) {
    // Null shapes are ignored.
    if (shape != nullptr) {
        subshapes_.push_back(shape);
    }
}

void CompositeShape::add(const CompositeShape& comp_shape) {
    // Share each subshape pointer. add() still drops nulls.
    for (const auto& shape : comp_shape.subshapes()) {
        add(shape);
    }
}

void CompositeShape::clear() { subshapes_.clear(); }

}  // namespace rj_geometry
