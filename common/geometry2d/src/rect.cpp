#include <geometry2d/rect.h>
#include <geometry2d/point.h>
#include <geometry2d/segment.h>
#include <constants.h>

namespace geometry2d {
using namespace std;

//constants used for the rect-segment intersection
const int INSIDE = 0x00; // 0000
const int LEFT = 0x01;   // 0001
const int RIGHT = 0x02;  // 0010
const int BOTTOM = 0x04; // 0100
const int TOP = 0x08;    // 1000

Shape* Rect::clone() const { return new Rect(*this); }

bool Rect::intersects(const Rect& other) const {
    return !(other.maxx() < minx() || other.minx() > maxx() ||
             other.maxy() < miny() || other.miny() > maxy());
}

int Rect::CohenSutherlandOutCode(const Point& other) const{
    int code;
    double x;
    double y;
    x = other.x();
    y = other.y();

    code = INSIDE;          // initialised as being inside of [[clip window]]
    if (x < minx()) {       // to the left of clip window
        code |= LEFT;
    } else if (x > maxx()) {  // to the right of clip window
        code |= RIGHT;
    }
    if (y < miny()) {  // below the clip window
        code |= BOTTOM;
    } else if (y > maxy()) {  // above the clip window
        code |= TOP;
    }

    return code;
}

std::tuple<bool, std::vector<Point> > Rect::intersects(const Segment& other) const{

    //Code aggressively borrowed from wikipedia entry Cohen-Sutherland Line Clipping
    Point p0 = other.pt[0];
    double x0 = p0.x();
    double y0 = p0.y();
    Point p1 = other.pt[1];
    double x1 = p1.x();
    double y1 = p1.y();

    int outcode0 = CohenSutherlandOutCode(p0);
    int outcode1 = CohenSutherlandOutCode(p1);

    std::vector<Point> intersectionPoints;
    bool accept = false;
    while (true) {
        if ((outcode0 | outcode1) == 0) {
            // bitwise OR is 0: both points inside window; trivially accept and exit loop
            accept = true;
            break;
        }
        if ((outcode0 & outcode1) != 0) {
            // bitwise AND is not 0: both points share an outside zone (LEFT, RIGHT, TOP,
            // or BOTTOM), so both must be outside window; exit loop (accept is false)
            break;
        }
        // failed both tests, so calculate the line segment to clip
        // from an outside point to an intersection with clip edge
        double x;
        double y;

        // At least one endpoint is outside the clip rectangle; pick it.
        int outcodeOut = outcode0 != 0 ? outcode0 : outcode1;

        // Now find the intersection point;
        // use formulas:
        //   slope = (y1 - y0) / (x1 - x0)
        //   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
        //   y = y0 + slope * (xm - x0), where xm is xmin or xmax
        // No need to worry about divide-by-zero because, in each case, the
        // outcode bit being tested guarantees the denominator is non-zero

        if ((outcodeOut & TOP) != 0) {  // point is above the clip window
            x = x0 + (x1 - x0) * (maxy() - y0) / (y1 - y0);
            y = maxy();
        } else if ((outcodeOut & BOTTOM) !=
                   0) {  // point is below the clip window
            x = x0 + (x1 - x0) * (miny() - y0) / (y1 - y0);
            y = miny();
        } else if ((outcodeOut & RIGHT) !=
                   0) {  // point is to the right of clip window
            y = y0 + (y1 - y0) * (maxx() - x0) / (x1 - x0);
            x = maxx();
        } else if ((outcodeOut & LEFT) !=
                   0) {  // point is to the left of clip window
            y = y0 + (y1 - y0) * (minx() - x0) / (x1 - x0);
            x = minx();
        }

        // Now we move outside point to intersection point to clip
        // and get ready for next pass.
        if (outcodeOut == outcode0) {
            x0 = x;
            y0 = y;
            Point pt = Point(x0, y0);
            outcode0 = CohenSutherlandOutCode(pt);
            // Save point iff it is inside the Rect
            if (outcode0 == INSIDE) {
                intersectionPoints.push_back(pt);
            }
        } else {
            x1 = x;
            y1 = y;
            Point pt = Point(x1, y1);
            outcode1 = CohenSutherlandOutCode(pt);
            // Save point iff it is inside the Rect
            if (outcode1 == INSIDE) {
                intersectionPoints.push_back(pt);
            }
        }
    }
    return std::tuple<bool, std::vector<Point> >(accept,intersectionPoints);
}

std::vector<Point> Rect::corners(){
    std::vector<Point> tmp;
    tmp.emplace_back(minx(),miny());
    tmp.emplace_back(minx(),maxy());
    tmp.emplace_back(maxx(),maxy());
    tmp.emplace_back(maxx(),miny());
    return tmp;
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

bool Rect::hit(Point point) const { return nearPoint(point, Robot_Radius); }

void Rect::expand(Point p) {
    float _minx = minx();
    float _miny = miny();
    float _maxx = maxx();
    float _maxy = maxy();

    pt[0].x() = min(_minx, (float)p.x());
    pt[0].y() = min(_miny, (float)p.y());
    pt[1].x() = max(_maxx, (float)p.x());
    pt[1].y() = max(_maxy, (float)p.y());
}

void Rect::expand(const Rect& rect) {
    expand(rect.pt[0]);
    expand(rect.pt[1]);
}

void Rect::pad(float padding){
    float _minx = minx();
    float _miny = miny();
    float _maxx = maxx();
    float _maxy = maxy();
    pt[0].x() = _minx - padding;
    pt[0].y() = _miny - padding;
    pt[1].x() = _maxx + padding;
    pt[1].y() = _maxy + padding;
}

bool Rect::nearSegment(const Segment& seg, float threshold) const {
    Point p1 = seg.pt[0];
    Point p2 = seg.pt[1];

    // Simpler case if this rect is degenerate
    if (pt[0] == pt[1]) {
        // return pt[0].nearSegment(seg, threshold);
        return seg.nearPoint(pt[0], threshold);
    }

    // If either endpoint is inside this rect the the segment intersects it.
    if (this->containsPoint(p1) || this->containsPoint(p2)) {
        return true;
    }

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
        if (edge.nearPoint(p1, threshold) || edge.nearPoint(p2, threshold)) {
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

bool Rect::nearPoint(Point other, float threshold) const {
    // Simpler case if this rect is degenerate
    if (pt[0] == pt[1]) {
        return pt[0].distTo(other) <= threshold;
    }

    // If the point is inside this rect then it is near it.
    if (this->containsPoint(other)) {
        return true;
    }

    Point ur = Point(pt[1].x(), pt[0].y());
    Point ll = Point(pt[0].x(), pt[1].y());
    Segment edges[4] = {Segment(pt[0], ur), Segment(ur, pt[1]),
                        Segment(pt[0], ll), Segment(ll, pt[1])};

    // If any edge of this rect is near the point, then the point is near the
    // rect.
    for (Segment& edge : edges) {
        if (edge.nearPoint(other, threshold)) {
            return true;
        }
    }

    return false;
}

}  // namespace geometry2d
