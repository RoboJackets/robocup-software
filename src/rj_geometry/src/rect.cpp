#include "rj_geometry/rect.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include "rj_geometry/point.hpp"
#include "rj_geometry/segment.hpp"

namespace rj_geometry {

void Rect::expand(Point p) {
    corners_[0].x() = std::min(minx(), static_cast<float>(p.x()));
    corners_[0].y() = std::min(miny(), static_cast<float>(p.y()));
    corners_[1].x() = std::max(maxx(), static_cast<float>(p.x()));
    corners_[1].y() = std::max(maxy(), static_cast<float>(p.y()));
}

void Rect::expand(const Rect& rect) {
    std::array<Point, 2> corners = rect.corners();
    expand(corners[0]);
    expand(corners[1]);
}

void Rect::pad(float padding) {
    corners_[0].x() = minx() - padding;
    corners_[0].y() = miny() - padding;
    corners_[1].x() = maxx() + padding;
    corners_[1].y() = maxy() + padding;
}

bool Rect::contains_point(const Point& p) const {
    return (p.x() >= minx()) && (p.x() <= maxx()) && (p.y() >= miny()) && (p.y() <= maxy());
}

bool Rect::intersects_segment(const Segment& seg) const {
    if (contains_point(seg.pt[0]) || contains_point(seg.pt[1])) {
        return true;
    }
    return !get_segment_intersections(seg).empty();
}

bool Rect::near_point(const Point& p, float clearance) const {
    if (corners_[0] == corners_[1]) {
        return corners_[0].dist_to(p) <= clearance;
    }

    if (contains_point(p)) {
        return true;
    }

    auto dx = std::max<double>({0.0, p.x() - maxx(), minx() - p.x()});
    auto dy = std::max<double>({0.0, p.y() - maxy(), miny() - p.y()});

    double distance_to_point = std::sqrt(dx * dx + dy * dy);

    return distance_to_point < clearance;
}

bool Rect::near_segment(const Segment& seg, float clearance) const {
    Point p1 = seg.pt[0];
    Point p2 = seg.pt[1];

    if (corners_[0] == corners_[1]) {
        return seg.near_point(corners_[0], clearance);
    }

    if (contains_point(p1) || contains_point(p2)) {
        return true;
    }

    // If any corner of this rect is near the segment, then the segment is near this rect.
    Point ur = Point(corners_[1].x(), corners_[0].y());
    Point ll = Point(corners_[0].x(), corners_[1].y());
    if (seg.near_point(corners_[0], clearance) || seg.near_point(ur, clearance) ||
        seg.near_point(ll, clearance) || seg.near_point(corners_[1], clearance)) {
        return true;
    }

    Segment edges[4] = {Segment(corners_[0], ur), Segment(ur, corners_[1]),
                        Segment(corners_[0], ll), Segment(ll, corners_[1])};

    // If either endpoint of the segment is near an edge of the rect, then the segment is near this rect.
    for (Segment& edge : edges) {
        if (edge.near_point(p1, clearance) || edge.near_point(p2, clearance)) {
            return true;
        }
    }

    // If any edge of this rect intersects the segment, then the segment is near this rect.
    for (Segment& edge : edges) {
        if (seg.intersects(edge)) {
            return true;
        }
    }

    return false;
}

namespace {

void append_unique(std::vector<Point>& points, const Point& point) {
    for (const Point& existing : points) {
        if (existing.nearly_equals(point)) {
            return;
        }
    }
    points.push_back(point);
}

}  // namespace

std::vector<Point> Rect::get_segment_intersections(const Segment& seg) const {
    // Boundary contacts only. A segment that stays strictly inside this
    // rectangle produces an empty list; intersects_segment reports that case
    // from the endpoints.
    //
    // Walk the four edges in order: left, top, right, bottom. Each edge is
    // tested with Segment::intersection_points, which returns one point for a
    // crossing and both endpoints of a collinear overlap. A corner belongs to
    // two edges, so append_unique keeps a single copy when both edges report it.
    const Point corners[4] = {Point(minx(), miny()), Point(minx(), maxy()), Point(maxx(), maxy()),
                              Point(maxx(), miny())};

    std::vector<Point> points;
    for (int i = 0; i < 4; ++i) {
        const Segment edge(corners[i], corners[(i + 1) % 4]);
        for (const Point& hit : seg.intersection_points(edge)) {
            append_unique(points, hit);
        }
    }
    return points;
}

std::unique_ptr<Rect> Rect::clone() const { return std::unique_ptr<Rect>(clone_impl()); }

Rect* Rect::clone_impl() const { return new Rect(*this); }

}  // namespace rj_geometry
