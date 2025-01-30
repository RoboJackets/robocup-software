//
// Created by matt on 7/19/15.
//

#include <rj_geometry/arc.hpp>

using namespace std;

namespace rj_geometry {

vector<Point> Arc::intersects(const Line& line) const {
    // http://mathworld.wolfram.com/Circle2d-LineIntersection.html
    const auto cx = static_cast<float>(center_.x());
    const auto cy = static_cast<float>(center_.y());

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
        return {};
    }
    if (disc == 0) {
        // One point
        Point p{det * dy / drsq + cx, -det * dx / drsq + cy};

        // TODO ashaw596 CHECK this part. Seems suspect since angle_to is really
        // angle_between
        auto angle = center_.angle_between(p);
        if (angle > start_angle_ && angle < end_angle_) {
            return {p};
        }
        return {};
    }
    float sgn_dy = (dy < 0) ? -1 : 1;
    float sqrt_disc = sqrtf(disc);
    float abs_dy = fabs(dy);

    Point a{(det * dy + sgn_dy * dx * sqrt_disc) / drsq + cx,
            (-det * dx + abs_dy * sqrt_disc) / drsq + cy};
    Point b{(det * dy - sgn_dy * dx * sqrt_disc) / drsq + cx,
            (-det * dx - abs_dy * sqrt_disc) / drsq + cy};

    vector<Point> results;

    auto angle = (a - center_).angle();
    if (angle > start_angle_ && angle < end_angle_) {
        results.push_back(a);
    }
    angle = (b - center_).angle();
    if (angle > start_angle_ && angle < end_angle_) {
        results.push_back(b);
    }
    return results;
}

vector<Point> Arc::intersects(const Segment& segment) const {
    // get candidate intersections, pretending the segment extends infinitely
    vector<Point> candidates = intersects(Line(segment.pt[0], segment.pt[1]));
    // filter out candidates not on segment
    auto iter = candidates.begin();
    while (iter != candidates.end()) {
        auto& candidate = *iter;
        if (segment.dist_to(candidate) == 0.0f) {
            iter++;
        } else {
            iter = candidates.erase(iter);
        }
    }
    return candidates;
}

}  // namespace rj_geometry
