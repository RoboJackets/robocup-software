#include <rj_geometry/polygon.hpp>

namespace rj_geometry {

Polygon::Polygon(const Rect& rect) {
    // Boundary order: pt[0], (pt[1].x, pt[0].y), pt[1], then (pt[0].x, pt[1].y).
    vertices.resize(4);
    vertices[0] = rect.pt[0];
    vertices[1] = Point(rect.pt[1].x(), rect.pt[0].y());
    vertices[2] = rect.pt[1];
    vertices[3] = Point(rect.pt[0].x(), rect.pt[1].y());
}

Polygon::Polygon(std::vector<Point> verts) { vertices = std::move(verts); }

std::unique_ptr<Polygon> Polygon::clone() const {
    return std::unique_ptr<Polygon>(clone_impl());
}

Polygon* Polygon::clone_impl() const { return new Polygon(*this); }

void Polygon::init(const Segment& seg, float r, float length) {
    Point dir;
    if (length != 0.0f) {
        dir = seg.delta() / length;
    } else {
        // Degenerate segment, so direction doesn't matter as long as it's a unit vector.
        dir = Point(1, 0);
    }

    // v extends r past each endpoint. u is the perpendicular offset of length r.
    Point v = dir * r;
    Point u = v.perp_ccw();

    // Four corners: the start point shifted by -v +/- u,
    // then the end point shifted by +v +/- u.
    vertices.resize(4);
    vertices[0] = seg.pt[0] - u - v;
    vertices[1] = seg.pt[0] + u - v;
    vertices[2] = seg.pt[1] + u + v;
    vertices[3] = seg.pt[1] - u + v;
}

Rect Polygon::bbox() const {
    // Seed the bounds with the first vertex, then expand to the rest.
    Rect rect(vertices[0]);

    for (unsigned int i = 1; i < vertices.size(); ++i) {
        rect.expand(vertices[i]);
    }

    return rect;
}

bool Polygon::intersects(const Rect& rect) const {
    // Use the rectangle's four-corner polygon.
    return intersects(Polygon(rect));
}

bool Polygon::intersects(const Polygon& other) const {
    // True when either polygon contains a vertex of the other.
    return contains_vertex(other) || other.contains_vertex(*this);
}

bool Polygon::contains_vertex(const Polygon& other) const {
    // True if any vertex of other lies inside this polygon.
    for (auto vertice : other.vertices) {
        if (contains_point(vertice)) {
            return true;
        }
    }

    return false;
}

bool Polygon::near_point(const Point& pt, float threshold) const {
    if (contains_point(pt)) {
        return true;
    }

    // Also true when pt is within threshold of any edge, including the closing edge.
    unsigned int i = vertices.size() - 1;
    for (unsigned int j = 0; j < vertices.size(); ++j) {
        Segment edge(vertices[i], vertices[j]);
        if (edge.near_point(pt, threshold)) {
            return true;
        }

        i = j;
    }

    return false;
}

bool Polygon::near_segment(const Segment& seg, float threshold) const {
    // An endpoint inside the polygon is a hit.
    if (contains_point(seg.pt[0]) || contains_point(seg.pt[1])) {
        return true;
    }

    // Otherwise test each edge, including the closing edge, against seg.
    unsigned int i = vertices.size() - 1;
    for (unsigned int j = 0; j < vertices.size(); ++j) {
        Segment edge(vertices[i], vertices[j]);
        if (edge.near_segment(seg, threshold)) {
            return true;
        }

        i = j;
    }

    return false;
}

bool Polygon::intersects_segment(const Segment& seg) const {
    // An empty polygon has no interior and no edges.
    if (vertices.empty()) {
        return false;
    }
    // Either endpoint inside the polygon is an intersection.
    if (contains_point(seg.pt[0]) || contains_point(seg.pt[1])) {
        return true;
    }
    // Otherwise an edge must cross the segment.
    return !get_segment_intersections(seg).empty();
}

std::vector<Point> Polygon::get_segment_intersections(const Segment& seg) const {
    std::vector<Point> points;
    // A polygon needs at least one edge.
    if (vertices.size() < 2) {
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
    for (unsigned int i = 0; i < vertices.size(); ++i) {
        const Segment edge(vertices[i], vertices[(i + 1) % vertices.size()]);
        for (const Point& hit : seg.intersection_points(edge)) {
            add_point(hit);
        }
    }
    return points;
}

bool Polygon::contains_point(const Point& pt) const {
    // FIXME (Ben) - Replace this with the optimized wrap-number test.

    // http://www.geometryalgorithms.com/Archive/algorithm_0103/algorithm_0103.htm
    // isLeft in their description does not actually indicate if a point is
    // on the -X side of a line.  It is the same as my Line::point_side.
    //
    // Simple explanation:
    // Consider a ray along +X starting at this point. Go around the polygon and
    // look at every intersection of an edge with this ray. Since you are
    // following the polygon's edges in order, if this point is outside the
    // polygon the ray will intersect the same number of up-going and down-going
    // edges.
    //
    // Consider the case of a convex polygon with this point outside: The ray
    // will intersect exactly one up-going edge and one down-going edge. For a
    // non-convex polygon, the ray will intersect zero or more additional pairs
    // of up-going and down-going edges.
    //
    // Increment count for each up-going edge and decrement for each down-going
    // edge. count==0 iff the point is outside.
    //
    // This handles cases where the ray intersects vertices too. Only the
    // endpoint with the smallest Y-value is considered. If the ray is outside
    // the polygon, then both edges will be counted because one will be up-going
    // and one down-going at the vertex. If the ray is inside the polygon, both
    // edges are going in the same direction so the vertex will only be checked
    // for one of them.

    int count = 0;
    unsigned int i = vertices.size() - 1;
    for (unsigned int j = 0; j < vertices.size(); j++) {
        const Point& p1 = vertices[i];
        const Point& p2 = vertices[j];
        i = j;

        if (p1.y() <= pt.y()) {
            if (p2.y() > pt.y()) {
                // Edge is going up
                if (Line(p1, p2).point_side(pt) > 0) {
                    ++count;
                }
            }
        } else {
            if (p2.y() <= pt.y()) {
                // Edge is going down
                if (Line(p1, p2).point_side(pt) < 0) {
                    --count;
                }
            }
        }
    }

    return count != 0;
}

}  // namespace rj_geometry
