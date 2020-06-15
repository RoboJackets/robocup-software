
#include "geometry2d/Segment.hpp"

#include "geometry2d/Util.hpp"

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
    const auto dist = static_cast<float>(nearestPoint(other).distTo(other));
    if (nearlyEqual(dist, 0)) {
        return 0;
    }
    return dist;
}

// Basic Tests
bool Segment::intersects(const Segment& other, Point* intr) const {
    // From Mathworld:
    // http://mathworld.wolfram.com/Line2d-Line2dIntersection.html

    const auto x1 = static_cast<float>(pt[0].x());
    const auto y1 = static_cast<float>(pt[0].y());
    const auto x2 = static_cast<float>(pt[1].x());
    const auto y2 = static_cast<float>(pt[1].y());
    const auto x3 = static_cast<float>(other.pt[0].x());
    const auto y3 = static_cast<float>(other.pt[0].y());
    const auto x4 = static_cast<float>(other.pt[1].x());
    const auto y4 = static_cast<float>(other.pt[1].y());

    const float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (denom == 0) {
        return false;
    }

    float deta = x1 * y2 - y1 * x2;
    float detb = x3 * y4 - y3 * x4;

    float ix = (deta * (x3 - x4) - (x1 - x2) * detb) / denom;
    float iy = (deta * (y3 - y4) - (y1 - y2) * detb) / denom;

    Point ip(ix, iy);

    Point da = delta();
    const float ta = static_cast<float>((ip - pt[0]).dot(da) / da.magsq());

    Point db = other.delta();
    const float tb =
        static_cast<float>((ip - other.pt[0]).dot(db) / db.magsq());

    if (ta < 0 || ta > 1 || tb < 0 || tb > 1) {
        return false;
    }

    if (intr != nullptr) {
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
    }
    return false;
}

bool Segment::nearPoint(const Point& point, float threshold) const {
    return distTo(point) <= threshold;
}

// Fixed/Audited by Albert.
// Some simple Tests
Point Segment::nearestPoint(const Point& p) const {
    // http://stackoverflow.com/a/1501725

    const double magsq = delta().magsq();
    if (magsq == 0) {
        return pt[0];
    }

    const double t = delta().dot(p - pt[0]) / magsq;

    if (t <= 0) {
        return pt[0];
    }
    if (t >= 1) {
        return pt[1];
    }
    return pt[0] + delta() * t;
}

Point Segment::nearestPoint(const Line& l) const {
    Point intersection;
    if (intersects(l, &intersection)) {
        return intersection;
    }
    if (l.distTo(pt[0]) < l.distTo(pt[1])) {
        return pt[0];
    }
    return pt[1];
}

bool Segment::nearSegment(const Segment& other, float threshold) const {
    return intersects(other) || other.nearPoint(pt[0], threshold) ||
           other.nearPoint(pt[1], threshold) ||
           nearPoint(other.pt[0], threshold) ||
           nearPoint(other.pt[1], threshold);
}
}  // namespace Geometry2d
