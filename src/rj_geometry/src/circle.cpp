#include <memory>
#include <vector>
#include <cstdlib>
#include <cmath>

#include "rj_geometry/circle.hpp"
#include "rj_geometry/segment.hpp"
#include "rj_geometry/point.hpp"


namespace rj_geometry {

bool Circle::contains_point(const Point& p) const {
    return (p - center()).mag() <= radius();
}

bool Circle::near_point(const Point& p, float clearance) const {
    return (p - center()).mag() <= radius() + clearance;
}

bool Circle::intersects_segment(const Segment& seg) const {
    return seg.near_point(center(), radius());
}

bool Circle::near_segment(const Segment& seg, float clearance) const {
    return seg.near_point(center(), radius() + clearance);
}

std::vector<Point> Circle::get_segment_intersections(const Segment& seg) const {
    std::vector<Point> result;

    const Point d = seg.delta();
    const Point f = seg.pt[0] - center();
    const float r = radius();

    const float a = d.dot(d);
    const float b = 2.0f * f.dot(d);
    const float c = f.dot(f) - r * r;

    // Degenerate segment (both endpoints coincide): intersects only if the point is on the circle
    constexpr float kEps = 1e-6f;
    if (a < kEps) {
        if (std::abs(c) < kEps) {
            result.push_back(seg.pt[0]);
        }
        return result;
    }

    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) {
        return result;  // line misses the circle
    }

    // Parameter t along the segment: P(t) = pt[0] + t * d, valid for t in [0, 1]
    auto add_if_on_segment = [&](float t) {
        if (t >= 0.0f && t <= 1.0f) {
            result.push_back(seg.pt[0] + d * t);
        }
    };

    if (disc < kEps) {
        // Tangent: a single intersection point
        add_if_on_segment(-b / (2.0f * a));
    } else {
        const float sqrt_disc = std::sqrt(disc);
        add_if_on_segment((-b - sqrt_disc) / (2.0f * a));  // entry point (closer to pt[0])
        add_if_on_segment((-b + sqrt_disc) / (2.0f * a));  // exit point
    }

    return result;
}

std::unique_ptr<Circle> Circle::clone() const { return std::unique_ptr<Circle>(clone_impl()); }
Circle* Circle::clone_impl() const { return new Circle(*this); }

}  // namespace rj_geometry
