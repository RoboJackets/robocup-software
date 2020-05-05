#include "Line.hpp"

#include <cmath>

#include "Circle.hpp"
#include "Point.hpp"
#include "Segment.hpp"
#include "Util.hpp"
namespace Geometry2d {

Line::Line(const Segment& segment) : Line(segment.pt[0], segment.pt[1]) {}

// Uses Tested Function
bool Line::intersects(const Line& other, Point* intersection) const { return intersects(*this, other, intersection); }

// Has Simple Test
bool Line::intersects(const Line& line1, const Line& line2, Point* intersection) {
    // From Mathworld:
    // http://mathworld.wolfram.com/Line-LineIntersection.html

    const auto x1 = static_cast<float>(line1.pt[0].x());
    const auto y1 = static_cast<float>(line1.pt[0].y());
    const auto x2 = static_cast<float>(line1.pt[1].x());
    const auto y2 = static_cast<float>(line1.pt[1].y());
    const auto x3 = static_cast<float>(line2.pt[0].x());
    const auto y3 = static_cast<float>(line2.pt[0].y());
    const auto x4 = static_cast<float>(line2.pt[1].x());
    const auto y4 = static_cast<float>(line2.pt[1].y());

    const float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (denom == 0) {
        return false;
    }

    if (intersection != nullptr) {
        float deta = x1 * y2 - y1 * x2;
        float detb = x3 * y4 - y3 * x4;

        intersection->x() = (deta * (x3 - x4) - (x1 - x2) * detb) / denom;
        intersection->y() = (deta * (y3 - y4) - (y1 - y2) * detb) / denom;
    }
    return true;
}

// Has Simple Test
float Line::distTo(Point other) const {
    Point delta = pt[1] - pt[0];
    const auto top = static_cast<float>(delta.x() * (pt[0].y() - other.y()) - (pt[0].x() - other.x()) * delta.y());

    return std::fabs(top) / static_cast<float>(delta.mag());
}

// Very Simple Test
Point Line::nearestPoint(Point p) const {
    Point v_hat = delta().normalized();
    return pt[0] + v_hat * v_hat.dot(p - pt[0]);
}

// Fixed/Audited By Albert
// Has Simple Test
bool Line::intersects(const Circle& circle, Point* p1, Point* p2) const {
    // http://mathworld.wolfram.com/Circle-LineIntersection.html
    const auto x1 = static_cast<float>(pt[0].x() - circle.center.x());
    const auto x2 = static_cast<float>(pt[1].x() - circle.center.x());

    const auto y1 = static_cast<float>(pt[0].y() - circle.center.y());
    const auto y2 = static_cast<float>(pt[1].y() - circle.center.y());

    const float dx = x2 - x1;
    const float dy = y2 - y1;
    const float dr2 = dx * dx + dy * dy;
    const float r = circle.radius();

    if (dx == 0 && dy == 0) {
        return false;
    }

    float det = x1 * y2 - x2 * y1;

    float descr = r * r * dr2 - det * det;

    if (descr < 0) {
        return false;
    }

    float common = std::sqrt(descr);

    float xPart1 = det * dy;
    float signDy = dy < 0 ? -1 : 1;

    float xPart2 = signDy * dx * common;

    float yPart1 = -det * dx;
    float yPart2 = std::fabs(dy) * common;

    if (p1 != nullptr) {
        float x = xPart1 + xPart2;
        float y = yPart1 + yPart2;
        *p1 = Point(x / dr2 + circle.center.x(), y / dr2 + circle.center.y());
    }

    if (p2 != nullptr) {
        float x = xPart1 - xPart2;
        float y = yPart1 - yPart2;
        *p2 = Point(x / dr2 + circle.center.x(), y / dr2 + circle.center.y());
    }

    return true;
}

bool Line::intersects(const Segment& other, Point* intersection) const { return other.intersects(*this, intersection); }
}  // namespace Geometry2d
