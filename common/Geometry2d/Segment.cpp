
#include "Segment.hpp"
#include "Util.hpp"

using namespace std;

namespace Geometry2d {

Rect Segment::bbox() const {
    Rect bbox;

    bbox.pt[0].x() = min(pt[0].x(), pt[1].x());
    bbox.pt[0].y() = min(pt[0].y(), pt[1].y());
    bbox.pt[1].x() = max(pt[0].x(), pt[1].x());
    bbox.pt[1].y() = max(pt[0].y(), pt[1].y());

    return bbox;
}

float Segment::distTo(const Point& other) const {
    float dist = nearestPoint(other).distTo(other);
    if (nearlyEqual(dist, 0)) {
        return 0;
    }
    return dist;
}

// Basic Tests
bool Segment::intersects(const Segment& other, Point* intr) const {
    // From Mathworld:
    // http://mathworld.wolfram.com/Line2d-Line2dIntersection.html

    float x1 = pt[0].x();
    float y1 = pt[0].y();
    float x2 = pt[1].x();
    float y2 = pt[1].y();
    float x3 = other.pt[0].x();
    float y3 = other.pt[0].y();
    float x4 = other.pt[1].x();
    float y4 = other.pt[1].y();

    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (denom == 0) {
        return false;
    }

    float deta = x1 * y2 - y1 * x2;
    float detb = x3 * y4 - y3 * x4;

    float ix = (deta * (x3 - x4) - (x1 - x2) * detb) / denom;
    float iy = (deta * (y3 - y4) - (y1 - y2) * detb) / denom;

    Point ip(ix, iy);

    Point da = delta();
    float ta = (ip - pt[0]).dot(da) / da.magsq();

    Point db = other.delta();
    float tb = (ip - other.pt[0]).dot(db) / db.magsq();

    if (ta < 0 || ta > 1 || tb < 0 || tb > 1) {
        return false;
    }

    if (intr) {
        intr->x() = ix;
        intr->y() = iy;
    }

    return true;
}

bool Segment::intersects(const Circle& circle) const {
    return nearPoint(circle.center, circle.radius());
}

bool Segment::intersects(const Line& line, Point* intr) const {
    Point intersection_point;
    bool res = Line::intersects(Line(*this), line, &intersection_point);
    if (res && distTo(intersection_point) == 0) {
        if (intr != nullptr) {
            *intr = intersection_point;
        }
        return true;
    } else {
        return false;
    }
}

bool Segment::nearPoint(const Point& point, float threshold) const {
    return distTo(point) <= threshold;
}

// Fixed/Audited by Albert.
// Some simple Tests
Point Segment::nearestPoint(const Point& p) const {
    // http://stackoverflow.com/a/1501725

    const double magsq = delta().magsq();
    if (magsq == 0) return pt[0];

    const double t = delta().dot(p - pt[0]) / magsq;

    if (t <= 0) {
        return pt[0];
    } else if (t >= 1) {
        return pt[1];
    }
    return pt[0] + delta() * t;
}

Point Segment::nearestPoint(const Line& l) const {
    Point intersection;
    if (intersects(l, &intersection)) {
        return intersection;
    } else if (l.distTo(pt[0]) < l.distTo(pt[1])) {
        return pt[0];
    } else {
        return pt[1];
    }
}

bool Segment::nearSegment(const Segment& other, float threshold) const {
    return intersects(other) || other.nearPoint(pt[0], threshold) ||
           other.nearPoint(pt[1], threshold) ||
           nearPoint(other.pt[0], threshold) ||
           nearPoint(other.pt[1], threshold);
}
}  // namespace Geometry2d
