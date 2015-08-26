#include "Circle.hpp"
#include "Segment.hpp"
#include <Constants.hpp>

using namespace Geometry2d;

Shape* Circle::clone() const { return new Circle(*this); }

int Circle::intersects(Circle& other, Point* i) const {
    // http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
    // (Mathworld's solution uses inconvenient coordinates)
    //
    // This is rearranged to perform the sqrts after the number of intersection
    // points is known, so they can be omitted if the points are not needed or
    // if the circles don't intersect.

    float dsq = (center - other.center).magsq();
    if (dsq == 0) {
        // Concentric circles: no points or all points
        return 0;
    }

    float r0sq = radius_sq();
    float r1sq = other.radius_sq();

    float t = (r0sq - r1sq + dsq);
    float asq = t * t / (4 * dsq);
    float hsq = r0sq - asq;

    int n;
    if (hsq < 0) {
        return 0;
    } else if (hsq == 0) {
        n = 1;
    } else {
        n = 2;
    }

    if (i) {
        float x0 = center.x;
        float y0 = center.y;
        float x1 = other.center.x;
        float y1 = other.center.y;

        float a_over_d = sqrtf(asq / dsq);
        float h_over_d = sqrtf(hsq / dsq);

        float x2 = x0 + (x1 - x0) * a_over_d;
        float y2 = y0 + (y1 - y0) * a_over_d;

        i[0].x = x2 + (y1 - y0) * h_over_d;
        i[0].y = y2 - (x1 - x0) * h_over_d;

        if (n == 2) {
            i[1].x = x2 - (y1 - y0) * h_over_d;
            i[1].y = y2 + (x1 - x0) * h_over_d;
        }
    }

    return n;
}

int Circle::intersects(const Line& line, Point* i) const {
    // http://mathworld.wolfram.com/Circle2d-LineIntersection.html
    float cx = center.x;
    float cy = center.y;

    float x1 = line.pt[0].x - cx;
    float y1 = line.pt[0].y - cy;
    float x2 = line.pt[1].x - cx;
    float y2 = line.pt[1].y - cy;

    float dx = x2 - x1;
    float dy = y2 - y1;
    float drsq = dx * dx + dy * dy;
    float det = x1 * y2 - x2 * y1;

    float disc = radius_sq() * drsq - det * det;
    if (disc < 0) {
        // No intersection
        return 0;
    } else if (disc == 0) {
        // One point
        if (i) {
            i[0].x = det * dy / drsq + cx;
            i[0].y = -det * dx / drsq + cy;
        }

        return 1;
    } else {
        // Two points
        if (i) {
            float sgn_dy = (dy < 0) ? -1 : 1;
            float sqrt_disc = sqrtf(disc);
            float abs_dy = fabs(dy);

            i[0].x = (det * dy + sgn_dy * dx * sqrt_disc) / drsq + cx;
            i[0].y = (-det * dx + abs_dy * sqrt_disc) / drsq + cy;

            i[1].x = (det * dy - sgn_dy * dx * sqrt_disc) / drsq + cx;
            i[1].y = (-det * dx - abs_dy * sqrt_disc) / drsq + cy;
        }

        return 2;
    }
}

bool Circle::containsPoint(const Point& pt) const {
    return (pt - center).mag() < radius();
}

Point Circle::nearestPoint(const Geometry2d::Point& P) const {
    return (P - center).normalized() * _r + center;
}

bool Circle::tangentPoints(const Geometry2d::Point& src, Geometry2d::Point* p1,
                           Geometry2d::Point* p2) const {
    if (!p1 && !p2) {
        return false;
    }

    const float dist = src.distTo(center);

    if (dist < _r) {
        return false;
    } else if (dist == _r) {
        if (p1) {
            *p1 = src;
        }

        if (p2) {
            *p2 = src;
        }
    } else {
        // source is outside of circle
        const float theta = asin(_r / dist);

        if (p1) {
            Point final = center;
            final.rotate(src, theta);
            *p1 = final;
        }

        if (p2) {
            Point final = center;
            final.rotate(src, -theta);
            *p2 = final;
        }
    }

    return true;
}

bool Circle::hit(const Geometry2d::Point& pt) const {
    return pt.nearPoint(center, radius() + Robot_Radius);
}

bool Circle::hit(const Geometry2d::Segment& seg) const {
    return seg.nearPoint(center, radius() + Robot_Radius);
}
