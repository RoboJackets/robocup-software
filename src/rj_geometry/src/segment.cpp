
#include <algorithm>
#include <cmath>
#include <vector>

#include <rj_geometry/segment.hpp>
#include <rj_geometry/util.hpp>

using namespace std;

namespace rj_geometry {

Rect Segment::bbox() const {
    return Rect(Point(min(pt[0].x(), pt[1].x()), min(pt[0].y(), pt[1].y())),
                Point(max(pt[0].x(), pt[1].x()), max(pt[0].y(), pt[1].y())));
}

float Segment::dist_to(const Point& other) const {
    const auto dist = static_cast<float>(nearest_point(other).dist_to(other));
    if (nearly_equal(dist, 0)) {
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
    const float tb = static_cast<float>((ip - other.pt[0]).dot(db) / db.magsq());

    if (ta < 0 || ta > 1 || tb < 0 || tb > 1) {
        return false;
    }

    if (intr != nullptr) {
        intr->x() = ix;
        intr->y() = iy;
    }

    return true;
}

std::vector<Point> Segment::intersection_points(const Segment& other) const {
    Point crossing;
    if (intersects(other, &crossing)) {
        return {crossing};
    }

    const Point direction = delta();
    const double length_sq = direction.magsq();
    constexpr double kOnLine = 1e-8;
    if (length_sq == 0.0) {
        if (other.near_point(pt[0], static_cast<float>(kOnLine))) {
            return {pt[0]};
        }
        return {};
    }

    const auto cross = [](const Point& a, const Point& b) {
        return a.x() * b.y() - a.y() * b.x();
    };
    const double length = std::sqrt(length_sq);
    // Distance from each of the other segment's endpoints to this line.
    const double dist0 = std::abs(cross(direction, other.pt[0] - pt[0])) / length;
    const double dist1 = std::abs(cross(direction, other.pt[1] - pt[0])) / length;
    if (dist0 > kOnLine || dist1 > kOnLine) {
        return {};
    }

    const auto parameter = [&](const Point& point) {
        return direction.dot(point - pt[0]) / length_sq;
    };
    const double a = parameter(other.pt[0]);
    const double b = parameter(other.pt[1]);
    const double start = std::max(0.0, std::min(a, b));
    const double end = std::min(1.0, std::max(a, b));
    if (start > end + kOnLine) {
        return {};
    }

    std::vector<Point> points;
    points.push_back(pt[0] + direction * start);
    if (end > start + kOnLine) {
        points.push_back(pt[0] + direction * end);
    }
    return points;
}

bool Segment::intersects(const Circle& circle) const {
    return near_point(circle.center(), circle.radius());
}

bool Segment::intersects(const Line& line, Point* intr) const {
    Point intersection_point;
    bool res = Line::intersects(Line(*this), line, &intersection_point);
    if (res && dist_to(intersection_point) == 0) {
        if (intr != nullptr) {
            *intr = intersection_point;
        }
        return true;
    }
    return false;
}

bool Segment::near_point(const Point& point, float threshold) const {
    return dist_to(point) <= threshold;
}

// Fixed/Audited by Albert.
// Some simple Tests
Point Segment::nearest_point(const Point& p) const {
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

Point Segment::nearest_point(const Line& l) const {
    Point intersection;
    if (intersects(l, &intersection)) {
        return intersection;
    }
    if (l.dist_to(pt[0]) < l.dist_to(pt[1])) {
        return pt[0];
    }
    return pt[1];
}

bool Segment::near_segment(const Segment& other, float threshold) const {
    return intersects(other) || other.near_point(pt[0], threshold) ||
           other.near_point(pt[1], threshold) || near_point(other.pt[0], threshold) ||
           near_point(other.pt[1], threshold);
}
}  // namespace rj_geometry
