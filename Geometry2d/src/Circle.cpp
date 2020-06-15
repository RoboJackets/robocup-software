#include "geometry2d/Circle.hpp"
#include "geometry2d/Segment.hpp"
#include <rj_constants/constants.hpp>

namespace Geometry2d {

Shape* Circle::clone() const { return new Circle(*this); }

int Circle::intersects(Circle& other, Point* i) const {
    // http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
    // (Mathworld's solution uses inconvenient coordinates)
    //
    // This is rearranged to perform the sqrts after the number of intersection
    // points is known, so they can be omitted if the points are not needed or
    // if the circles don't intersect.

    float dsq = static_cast<float>((center - other.center).magsq());
    if (dsq == 0) {
        // Concentric circles: no points or all points
        return 0;
    }

    const float r0sq = radius_sq();
    const float r1sq = other.radius_sq();

    const float t = (r0sq - r1sq + dsq);
    const float asq = t * t / (4 * dsq);
    const float hsq = r0sq - asq;

    int n;
    if (hsq < 0) {
        return 0;
    }
    if (hsq == 0) {
        n = 1;
    } else {
        n = 2;
    }

    if (i != nullptr) {
        const auto x0 = static_cast<float>(center.x());
        const auto y0 = static_cast<float>(center.y());
        const auto x1 = static_cast<float>(other.center.x());
        const auto y1 = static_cast<float>(other.center.y());

        float a_over_d = sqrtf(asq / dsq);
        float h_over_d = sqrtf(hsq / dsq);

        float x2 = x0 + (x1 - x0) * a_over_d;
        float y2 = y0 + (y1 - y0) * a_over_d;

        i[0].x() = x2 + (y1 - y0) * h_over_d;
        i[0].y() = y2 - (x1 - x0) * h_over_d;

        if (n == 2) {
            i[1].x() = x2 - (y1 - y0) * h_over_d;
            i[1].y() = y2 + (x1 - x0) * h_over_d;
        }
    }

    return n;
}

int Circle::intersects(const Line& line, Point* i) const {
    // http://mathworld.wolfram.com/Circle2d-LineIntersection.html
    const auto cx = static_cast<float>(center.x());
    const auto cy = static_cast<float>(center.y());

    const auto x1 = static_cast<float>(line.pt[0].x() - cx);
    const auto y1 = static_cast<float>(line.pt[0].y() - cy);
    const auto x2 = static_cast<float>(line.pt[1].x() - cx);
    const auto y2 = static_cast<float>(line.pt[1].y() - cy);

    const float dx = x2 - x1;
    const float dy = y2 - y1;
    const float drsq = dx * dx + dy * dy;
    const float det = x1 * y2 - x2 * y1;

    const float disc = radius_sq() * drsq - det * det;
    if (disc < 0) {
        // No intersection
        return 0;
    }
    if (disc == 0) {
        // One point
        if (i != nullptr) {
            i[0].x() = det * dy / drsq + cx;
            i[0].y() = -det * dx / drsq + cy;
        }

        return 1;
    }
    // Two points
    if (i != nullptr) {
        float sgn_dy = (dy < 0) ? -1 : 1;
        float sqrt_disc = sqrtf(disc);
        float abs_dy = std::fabs(dy);

        i[0].x() = (det * dy + sgn_dy * dx * sqrt_disc) / drsq + cx;
        i[0].y() = (-det * dx + abs_dy * sqrt_disc) / drsq + cy;

        i[1].x() = (det * dy - sgn_dy * dx * sqrt_disc) / drsq + cx;
        i[1].y() = (-det * dx - abs_dy * sqrt_disc) / drsq + cy;
    }

    return 2;
}

bool Circle::containsPoint(Point pt) const {
    return (pt - center).mag() < radius();
}

Point Circle::nearestPoint(Point P) const {
    return (P - center).normalized() * _r + center;
}

bool Circle::tangentPoints(Point src, Point* p1, Point* p2) const {
    if ((p1 == nullptr) && (p2 == nullptr)) {
        return false;
    }

    const auto dist = static_cast<float>(src.distTo(center));

    if (dist < _r) {
        return false;
    }
    if (dist == _r) {
        if (p1 != nullptr) {
            *p1 = src;
        }

        if (p2 != nullptr) {
            *p2 = src;
        }
    } else {
        // source is outside of circle
        const float theta = std::asin(_r / dist);

        if (p1 != nullptr) {
            Point final = center;
            final.rotate(src, theta);
            *p1 = final;
        }

        if (p2 != nullptr) {
            Point final = center;
            final.rotate(src, -theta);
            *p2 = final;
        }
    }

    return true;
}

bool Circle::hit(Point pt) const {
    return pt.nearPoint(center, radius() + Robot_Radius);
}

bool Circle::hit(const Segment& seg) const {
    return seg.nearPoint(center, radius() + Robot_Radius);
}

bool Circle::nearPoint(Point pt, float threshold) const {
    return center.nearPoint(pt, threshold + radius());
}

}  // namespace Geometry2d
