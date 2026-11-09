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

// bool intersects_segment(const Segment& seg) const override; // TODO: use sutherland-cohen

// bool near_point(const Point& p, float clearance) const override;

// bool near_segment(const Segment& seg, float clearance) const override;

// std::vector<Point> get_segment_intersections(const Segment& seg) const override;

std::unique_ptr<Rect> Rect::clone() const { return std::unique_ptr<Rect>(clone_impl()); }

/** DEPRECATE AND REMOVE ALL REFERENCES! Old version of what is now split into intersects_segment and get_segment_intersections */
std::tuple<bool, std::vector<Point> > Rect::intersects(const Segment& other) const {
    // Code aggressively borrowed from wikipedia entry Cohen-Sutherland Line
    // Clipping
    Point p0 = other.pt[0];
    double x0 = p0.x();
    double y0 = p0.y();
    Point p1 = other.pt[1];
    double x1 = p1.x();
    double y1 = p1.y();

    int outcode0 = cohen_sutherland_out_code(p0);
    int outcode1 = cohen_sutherland_out_code(p1);

    std::vector<Point> intersection_points;
    bool accept = false;
    while (true) {
        if ((outcode0 | outcode1) == 0) {
            // bitwise OR is 0: both points inside window; trivially accept and
            // exit loop
            accept = true;
            break;
        }
        if ((outcode0 & outcode1) != 0) {
            // bitwise AND is not 0: both points share an outside zone (LEFT,
            // RIGHT, TOP, or BOTTOM), so both must be outside window; exit loop
            // (accept is false)
            break;
        }
        // failed both tests, so calculate the line segment to clip
        // from an outside point to an intersection with clip edge
        double x;
        double y;

        // At least one endpoint is outside the clip rectangle; pick it.
        int outcode_out = outcode0 != 0 ? outcode0 : outcode1;

        // Now find the intersection point;
        // use formulas:
        //   slope = (y1 - y0) / (x1 - x0)
        //   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
        //   y = y0 + slope * (xm - x0), where xm is xmin or xmax
        // No need to worry about divide-by-zero because, in each case, the
        // outcode bit being tested guarantees the denominator is non-zero

        if ((outcode_out & kTop) != 0) {  // point is above the clip window
            x = x0 + (x1 - x0) * (maxy() - y0) / (y1 - y0);
            y = maxy();
        } else if ((outcode_out & kBottom) != 0) {  // point is below the clip window
            x = x0 + (x1 - x0) * (miny() - y0) / (y1 - y0);
            y = miny();
        } else if ((outcode_out & kRight) != 0) {  // point is to the right of clip window
            y = y0 + (y1 - y0) * (maxx() - x0) / (x1 - x0);
            x = maxx();
        } else if ((outcode_out & kLeft) != 0) {  // point is to the left of clip window
            y = y0 + (y1 - y0) * (minx() - x0) / (x1 - x0);
            x = minx();
        }

        // Now we move outside point to intersection point to clip
        // and get ready for next pass.
        if (outcode_out == outcode0) {
            x0 = x;
            y0 = y;
            Point pt = Point(x0, y0);
            outcode0 = cohen_sutherland_out_code(pt);
            // Save point iff it is inside the Rect
            if (outcode0 == kInside) {
                intersection_points.push_back(pt);
            }
        } else {
            x1 = x;
            y1 = y;
            Point pt = Point(x1, y1);
            outcode1 = cohen_sutherland_out_code(pt);
            // Save point iff it is inside the Rect
            if (outcode1 == kInside) {
                intersection_points.push_back(pt);
            }
        }
    }
    return std::tuple<bool, std::vector<Point> >(accept, intersection_points);
}

Rect* Rect::clone_impl() const { return new Rect(*this); }

int Rect::cohen_sutherland_out_code(const Point& other) const {
    int code;
    double x;
    double y;
    x = other.x();
    y = other.y();

    code = kInside;
    if (x < minx()) {
        code |= kLeft;
    } else if (x > maxx()) {
        code |= kRight;
    }
    if (y < miny()) {
        code |= kBottom;
    } else if (y > maxy()) {
        code |= kTop;
    }

    return code;
}

// TODO below









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
    if (pt[0] == pt[1]) {
        // return pt[0].near_segment(seg, threshold);
        return seg.near_point(pt[0], threshold);
    }

    // If either endpoint is inside this rect the the segment intersects it.
    if (this->contains_point(p1) || this->contains_point(p2)) {
        return true;
    }

    // If any corner of this rect is near the segment,
    // then the segment is near this rect.
    Point ur = Point(pt[1].x(), pt[0].y());
    Point ll = Point(pt[0].x(), pt[1].y());
    if (seg.near_point(pt[0], threshold) || seg.near_point(ur, threshold) ||
        seg.near_point(ll, threshold) || seg.near_point(pt[1], threshold)) {
        return true;
    }

    Segment edges[4] = {Segment(pt[0], ur), Segment(ur, pt[1]), Segment(pt[0], ll),
                        Segment(ll, pt[1])};

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
    if (pt[0] == pt[1]) {
        return pt[0].dist_to(other) <= threshold;
    }

    // If the point is inside this rect then it is near it.
    if (this->contains_point(other)) {
        return true;
    }

    // Calculate the bounds of this rectangle.
    double min_x = std::min(pt[0].x(), pt[1].x());
    double max_x = std::max(pt[0].x(), pt[1].x());
    double min_y = std::min(pt[0].y(), pt[1].y());
    double max_y = std::max(pt[0].y(), pt[1].y());

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
