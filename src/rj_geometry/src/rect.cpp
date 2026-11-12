#include <algorithm>
#include <array>
#include <memory>
#include <vector>

#include "rj_geometry/point.hpp"
#include "rj_geometry/rect.hpp"
#include "rj_geometry/segment.hpp"

using namespace std;

namespace rj_geometry {

void Rect::expand(Point p) {
    corners_[0].x() = min(minx(), (float)p.x());
    corners_[0].y() = min(miny(), (float)p.y());
    corners_[1].x() = max(maxx(), (float)p.x());
    corners_[1].y() = max(maxy(), (float)p.y());
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

std::unique_ptr<Rect> Rect::clone() const { return std::unique_ptr<Rect>(clone_impl()); }

Rect* Rect::clone_impl() const { return new Rect(*this); }

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

bool Rect::intersects_segment(const Segment& seg) const {
    if (contains_point(seg.pt[0]) || contains_point(seg.pt[1])) {
        return true;
    }
    return !get_segment_intersections(seg).empty();
}

bool Rect::near_segment(const Segment& seg, float threshold) const {
    Point p1 = seg.pt[0];
    Point p2 = seg.pt[1];

    // Simpler case if this rect is degenerate
    if (corners_[0] == corners_[1]) {
        return seg.near_point(corners_[0], threshold);
    }

    // If either endpoint is inside this rect the the segment intersects it.
    if (this->contains_point(p1) || this->contains_point(p2)) {
        return true;
    }

    // If any corner of this rect is near the segment,
    // then the segment is near this rect.
    Point ur = Point(corners_[1].x(), corners_[0].y());
    Point ll = Point(corners_[0].x(), corners_[1].y());
    if (seg.near_point(corners_[0], threshold) || seg.near_point(ur, threshold) ||
        seg.near_point(ll, threshold) || seg.near_point(corners_[1], threshold)) {
        return true;
    }

    Segment edges[4] = {Segment(corners_[0], ur), Segment(ur, corners_[1]),
                        Segment(corners_[0], ll), Segment(ll, corners_[1])};

    // If either endpoint of the segment is near an edge of the rect, then the
    // segment is near this rect.
    for (Segment& edge : edges) {
        if (edge.near_point(p1, threshold) || edge.near_point(p2, threshold)) {
            return true;
        }
    }

    // If any edge of this rect intersects the segment, then the segment is near
    // this rect.
    for (Segment& edge : edges) {
        if (seg.intersects(edge)) {
            return true;
        }
    }

    return false;
}

bool Rect::near_point(const Point& other, float threshold) const {
    // Simpler case if this rect is degenerate
    if (corners_[0] == corners_[1]) {
        return corners_[0].dist_to(other) <= threshold;
    }

    // If the point is inside this rect then it is near it.
    if (this->contains_point(other)) {
        return true;
    }

    // Calculate the bounds of this rectangle.
    double min_x = std::min(corners_[0].x(), corners_[1].x());
    double max_x = std::max(corners_[0].x(), corners_[1].x());
    double min_y = std::min(corners_[0].y(), corners_[1].y());
    double max_y = std::max(corners_[0].y(), corners_[1].y());

    // Calculate the minimum distance to the rectangle in the x- and y-
    // directions. Example:
    //                                dx
    //         |                  | ----- *
    //         |                  |       | dy
    // --------*------------------*--------
    //         |                  |
    //         |                  |
    //         |                  |
    // --------*------------------*--------
    //         |                  |
    //         |                  |
    auto dx = std::max<double>({0.0, other.x() - max_x, min_x - other.x()});
    auto dy = std::max<double>({0.0, other.y() - max_y, min_y - other.y()});

    double distance_to_point = std::sqrt(dx * dx + dy * dy);

    return distance_to_point < threshold;
}

}  // namespace rj_geometry
