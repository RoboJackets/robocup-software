#include "rj_geometry/polygon.hpp"

#include <memory>
#include <vector>

#include "rj_geometry/line.hpp"
#include "rj_geometry/point.hpp"
#include "rj_geometry/segment.hpp"

namespace rj_geometry {

bool Polygon::contains_point(const Point& p) const {
    int count = 0;
    unsigned int i = vertices_.size() - 1;
    for (unsigned int j = 0; j < vertices_.size(); j++) {
        const Point& p1 = vertices_[i];
        const Point& p2 = vertices_[j];
        i = j;

        if (p1.y() <= p.y()) {
            if (p2.y() > p.y()) {
                // Edge is going up
                if (Line(p1, p2).point_side(p) > 0) {
                    ++count;
                }
            }
        } else {
            if (p2.y() <= p.y()) {
                // Edge is going down
                if (Line(p1, p2).point_side(p) < 0) {
                    --count;
                }
            }
        }
    }

    return count != 0;
}

bool Polygon::intersects_segment(const Segment& seg) const {
    // An empty polygon has no interior and no edges.
    if (vertices_.empty()) {
        return false;
    }
    // Either endpoint inside the polygon is an intersection.
    if (contains_point(seg.pt[0]) || contains_point(seg.pt[1])) {
        return true;
    }
    // Otherwise an edge must cross the segment.
    return !get_segment_intersections(seg).empty();
}

bool Polygon::near_point(const Point& p, float clearance) const {
    if (contains_point(p)) {
        return true;
    }

    // Also true when p is within clearance of any edge, including the closing edge.
    unsigned int i = vertices_.size() - 1;
    for (unsigned int j = 0; j < vertices_.size(); ++j) {
        Segment edge(vertices_[i], vertices_[j]);
        if (edge.near_point(p, clearance)) {
            return true;
        }

        i = j;
    }

    return false;
}

bool Polygon::near_segment(const Segment& seg, float clearance) const {
    // An endpoint inside the polygon is a hit.
    if (contains_point(seg.pt[0]) || contains_point(seg.pt[1])) {
        return true;
    }

    // Otherwise test each edge, including the closing edge, against seg.
    unsigned int i = vertices_.size() - 1;
    for (unsigned int j = 0; j < vertices_.size(); ++j) {
        Segment edge(vertices_[i], vertices_[j]);
        if (edge.near_segment(seg, clearance)) {
            return true;
        }

        i = j;
    }

    return false;
}

std::vector<Point> Polygon::get_segment_intersections(const Segment& seg) const {
    std::vector<Point> points;
    // A polygon needs at least one edge.
    if (vertices_.size() < 2) {
        return points;
    }

    // Adjacent edges both report a hit at a shared vertex. Keep one copy.
    auto add_point = [&points](const Point& point) {
        for (const Point& existing : points) {
            if (existing.nearly_equals(point)) {
                return;
            }
        }
        points.push_back(point);
    };

    // Walk every edge, including the closing edge from the last vertex to the first.
    for (unsigned int i = 0; i < vertices_.size(); ++i) {
        const Segment edge(vertices_[i], vertices_[(i + 1) % vertices_.size()]);
        for (const Point& hit : seg.intersection_points(edge)) {
            add_point(hit);
        }
    }
    return points;
}

std::unique_ptr<Polygon> Polygon::clone() const { return std::unique_ptr<Polygon>(clone_impl()); }

Polygon* Polygon::clone_impl() const { return new Polygon(*this); }

}  // namespace rj_geometry
