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

int Rect::CohenSutherlandCode(const Point& other) const{
    const int INSIDE = 0; // 0000
    const int LEFT = 1;   // 0001
    const int RIGHT = 2;  // 0010
    const int BOTTOM = 4; // 0100
    const int TOP = 8;    // 1000
    int code;
    double x,y;
    x = other.x();
    y = other.y();

    code = INSIDE;          // initialised as being inside of [[clip window]]
    if (x < minx())           // to the left of clip window
        code |= LEFT;
    else if (x > maxx())      // to the right of clip window
        code |= RIGHT;
    if (y < miny())           // below the clip window
        code |= BOTTOM;
    else if (y > maxy())      // above the clip window
        code |= TOP;

    return code;
}

bool Rect::intersects_(const Segment& other) const{
    //These constants are needed in here and in the CohenSutherlandCode method - is there
    //a place where we should make these definitions globally?
    const int INSIDE = 0; // 0000
    const int LEFT = 1;   // 0001
    const int RIGHT = 2;  // 0010
    const int BOTTOM = 4; // 0100
    const int TOP = 8;    // 1000

    //Code aggressively borrowed from wikipedia entry Cohen-Sutherland Line Clipping
    Point p0 = other.pt[0];
    double x0 = p0.x();
    double y0 = p0.y();
    Point p1 = other.pt[1];
    double x1 = p1.x();
    double y1 = p1.y();

    int outcode0 = CohenSutherlandCode(p0);
    int outcode1 = CohenSutherlandCode(p1);
    bool accept = false;

    while (true) {
        if (!(outcode0 | outcode1)) {
            // bitwise OR is 0: both points inside window; trivially accept and exit loop
            accept = true;
            break;
        } else if (outcode0 & outcode1) {
            // bitwise AND is not 0: both points share an outside zone (LEFT, RIGHT, TOP,
            // or BOTTOM), so both must be outside window; exit loop (accept is false)
            break;
        } else {
            // failed both tests, so calculate the line segment to clip
            // from an outside point to an intersection with clip edge
            double x, y;

            // At least one endpoint is outside the clip rectangle; pick it.
            int outcodeOut = outcode0 ? outcode0 : outcode1;

            // Now find the intersection point;
            // use formulas:
            //   slope = (y1 - y0) / (x1 - x0)
            //   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
            //   y = y0 + slope * (xm - x0), where xm is xmin or xmax
            // No need to worry about divide-by-zero because, in each case, the
            // outcode bit being tested guarantees the denominator is non-zero
            if (outcodeOut & TOP) {           // point is above the clip window
                x = x0 + (x1 - x0) * (maxy() - y0) / (y1 - y0);
                y = maxy();
            } else if (outcodeOut & BOTTOM) { // point is below the clip window
                x = x0 + (x1 - x0) * (miny() - y0) / (y1 - y0);
                y = miny();
            } else if (outcodeOut & RIGHT) {  // point is to the right of clip window
                y = y0 + (y1 - y0) * (maxx() - x0) / (x1 - x0);
                x = maxx();
            } else if (outcodeOut & LEFT) {   // point is to the left of clip window
                y = y0 + (y1 - y0) * (minx() - x0) / (x1 - x0);
                x = minx();
            }

            // Now we move outside point to intersection point to clip
            // and get ready for next pass.
            if (outcodeOut == outcode0) {
                x0 = x;
                y0 = y;
                Point tmp = Point(x0, y0);
                outcode0 = CohenSutherlandCode(tmp);
            } else {
                x1 = x;
                y1 = y;
                Point tmp = Point(x1, y1);
                outcode1 = CohenSutherlandCode(tmp);
            }
        }
    }

    return accept;

}



bool Rect::containsRect(const Rect& other) const {
    // return other.pt[0].inRect(*this) && other.pt[1].inRect(*this);
    return this->containsPoint(other.pt[0]) && this->containsPoint(other.pt[1]);
}

bool Rect::containsPoint(Point point) const {
    return point.x() >= minx() && point.x() <= maxx() && point.y() >= miny() &&
           point.y() <= maxy();
}

bool Rect::hit(const Segment& seg) const {
    return nearSegment(seg, Robot_Radius);
}

bool Rect::hit(Point pt) const { return nearPoint(pt, Robot_Radius); }

void Rect::expand(Point p) {
    pt[0].x() = min(pt[0].x(), p.x());
    pt[0].y() = min(pt[0].y(), p.y());
    pt[1].x() = max(pt[1].x(), p.x());
    pt[1].y() = max(pt[1].y(), p.y());
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
    Point ur = Point(pt[1].x(), pt[0].y());
    Point ll = Point(pt[0].x(), pt[1].y());
    if (seg.nearPoint(pt[0], threshold) || seg.nearPoint(ur, threshold) ||
        seg.nearPoint(ll, threshold) || seg.nearPoint(pt[1], threshold)) {
        return true;
    }

    Segment edges[4] = {Segment(pt[0], ur), Segment(ur, pt[1]),
                        Segment(pt[0], ll), Segment(ll, pt[1])};

    // If either endpoint of the segment is near an edge of the rect, then the
    // segment is near this rect.
    for (Segment& edge : edges) {
        if (edge.nearPoint(p1, threshold) || edge.nearPoint(p2, threshold))
            return true;
    }

    // If any edge of this rect intersects the segment, then the segment is near
    // this rect.
    for (Segment& edge : edges) {
        if (seg.intersects(edge)) return true;
    }

    return false;
}

bool Rect::nearPoint(Point other, float threshold) const {
    // Simpler case if this rect is degenerate
    if (pt[0] == pt[1]) return pt[0].distTo(other) <= threshold;

    // If the point is inside this rect then it is near it.
    if (this->containsPoint(other)) return true;

    Point ur = Point(pt[1].x(), pt[0].y());
    Point ll = Point(pt[0].x(), pt[1].y());
    Segment edges[4] = {Segment(pt[0], ur), Segment(ur, pt[1]),
                        Segment(pt[0], ll), Segment(ll, pt[1])};

    // If any edge of this rect is near the point, then the point is near the
    // rect.
    for (Segment& edge : edges) {
        if (edge.nearPoint(other, threshold)) return true;
    }

    return false;
}

}  // namespace Geometry2d
