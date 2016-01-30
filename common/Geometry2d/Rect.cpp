#include "Rect.hpp"
#include "Point.hpp"
#include "Segment.hpp"
#include <Constants.hpp>

using namespace std;

namespace Geometry2d {

Shape* Rect::clone() const { return new Rect(*this); }

bool Rect::intersects(const Rect& other) const {
    if (other.maxx() < minx() || other.minx() > maxx() ||
        other.maxy() < miny() || other.miny() > maxy()) {
        return false;
    } else {
        return true;
    }
}

bool Rect::containsRect(const Rect& other) const {
    // return other.pt[0].inRect(*this) && other.pt[1].inRect(*this);
    return this->containsPoint(other.pt[0]) && this->containsPoint(other.pt[1]);
}

bool Rect::containsPoint(Point point) const {
    float minx, miny, maxx, maxy;

    if (pt[0].x < pt[1].x) {
        minx = pt[0].x;
        maxx = pt[1].x;
    } else {
        minx = pt[1].x;
        maxx = pt[0].x;
    }

    if (pt[0].y < pt[1].y) {
        miny = pt[0].y;
        maxy = pt[1].y;
    } else {
        miny = pt[1].y;
        maxy = pt[0].y;
    }

    return point.x >= minx && point.x <= maxx && point.y >= miny &&
           point.y <= maxy;
}

bool Rect::hit(const Segment& seg) const {
    return containsPoint(seg.pt[0]) ||
           containsPoint(seg.pt[1]) ||
           seg.intersects(
               Segment(Point(minx(), miny()), Point(minx(), maxy()))) ||
           seg.intersects(
               Segment(Point(minx(), miny()), Point(maxx(), miny()))) ||
           seg.intersects(
               Segment(Point(minx(), maxy()), Point(maxx(), maxy()))) ||
           seg.intersects(
               Segment(Point(maxx(), maxy()), Point(maxx(), miny())));
}

bool Rect::hit(Point pt) const {
    return nearPoint(pt, Robot_Radius);
}

void Rect::expand(Point p) {
    pt[0].x = min(pt[0].x, p.x);
    pt[0].y = min(pt[0].y, p.y);
    pt[1].x = max(pt[1].x, p.x);
    pt[1].y = max(pt[1].y, p.y);
}

void Rect::expand(const Rect& rect) {
    expand(rect.pt[0]);
    expand(rect.pt[1]);
}

bool Rect::nearSegment(const Segment& seg, float threshold) const {
    Point p1 = seg.pt[0];
    Point p2 = seg.pt[1];

    // Simpler case if this rect is degenerate
    if (pt[0] == pt[1])
        // return pt[0].nearSegment(seg, threshold);
        return seg.nearPoint(pt[0], threshold);

    // If either endpoint is inside this rect the the segment intersects it.
    if (this->containsPoint(p1) || this->containsPoint(p2)) return true;

    // If any corner of this rect is near the segment,
    // then the segment is near this rect.
    Point ur = Point(pt[1].x, pt[0].y);
    Point ll = Point(pt[0].x, pt[1].y);
    if (seg.nearPoint(pt[0], threshold) || seg.nearPoint(ur, threshold) ||
        seg.nearPoint(ll, threshold) || seg.nearPoint(pt[1], threshold)) {
        return true;
    }

    Segment edge[4] = {Segment(pt[0], ur), Segment(ur, pt[1]),
                       Segment(pt[0], ll), Segment(ll, pt[1])};

    // If either endpoint of the segment is near an edge of the rect, then the
    // segment is near this rect.
    for (int i = 0; i < 4; i++) {
        if (edge[i].nearPoint(pt[0], threshold) ||
            edge[i].nearPoint(pt[1], threshold))
            return true;
    }

    // If any edge of this rect intersects the segment, then the segment is near
    // this rect.
    for (int i = 0; i < 4; i++) {
        if (seg.intersects(edge[i])) return true;
    }

    return false;
}

bool Rect::nearPoint(Point other, float threshold) const {
    // Simpler case if this rect is degenerate
    if (pt[0] == pt[1])
        return pt[0].distTo(other) <= threshold;

    // If the point is inside this rect then it is near it.
    if (this->containsPoint(other))
        return true;

    Point ur = Point(pt[1].x, pt[0].y);
    Point ll = Point(pt[0].x, pt[1].y);
    Segment edge[4] = {Segment(pt[0], ur), Segment(ur, pt[1]),
                       Segment(pt[0], ll), Segment(ll, pt[1])};

    // If any edge of this rect is near the point, then the point is near the
    // rect.
    for (int i = 0; i < 4; i++) {
        if (edge[i].nearPoint(other, threshold))
            return true;
    }

    return false;
}

}  // namespace Geometry2d
