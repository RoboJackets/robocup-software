//
// Created by matt on 7/19/15.
//

#include "Arc.hpp"

using namespace std;
using namespace Geometry2d;

vector<Point> Arc::intersects(const Line &line) const {
  // http://mathworld.wolfram.com/Circle2d-LineIntersection.html
  float cx = _center.x;
  float cy = _center.y;

  float x1 = line.pt[0].x - cx;
  float y1 = line.pt[0].y - cy;
  float x2 = line.pt[1].x - cx;
  float y2 = line.pt[1].y - cy;

  float dx = x2 - x1;
  float dy = y2 - y1;
  float drsq = dx * dx + dy * dy;
  float det = x1 * y2 - x2 * y1;

  float disc = radius_sq() * drsq - det * det;
  if (disc < 0)
  {
    // No intersection
    return {};
  } else if (disc == 0)
  {
    // One point
    Point p{det * dy / drsq + cx, -det * dx / drsq + cy};

    auto angle = _center.angleTo(p);
    if(angle > _start_angle && angle < _end_angle) {
      return {p};
    } else {
      return {};
    }

  } else {
    float sgn_dy = (dy < 0) ? -1 : 1;
    float sqrt_disc = sqrtf(disc);
    float abs_dy = fabs(dy);

    Point a{(det * dy + sgn_dy * dx * sqrt_disc) / drsq + cx,
            (-det * dx + abs_dy * sqrt_disc) / drsq + cy};
    Point b{(det * dy - sgn_dy * dx * sqrt_disc) / drsq + cx,
            (-det * dx - abs_dy * sqrt_disc) / drsq + cy};

    vector<Point> results;

    auto angle = _center.angleTo(a);
    if(angle > _start_angle && angle < _end_angle) {
      results.push_back(a);
    }
    angle = _center.angleTo(b);
    if(angle > _start_angle && angle < _end_angle) {
      results.push_back(b);
    }
    return results;
  }
}

vector<Point> Arc::intersects(const Segment &segment) const {
  // get candidate intersections, pretending the segment extends infinitely
  vector<Point> candidates = intersects(static_cast<const Line&>(segment));
  // filter out candidates not on segment
  auto iter = candidates.begin();
  while(iter != candidates.end()) {
    auto& candidate = *iter;
    if(segment.distTo(candidate) == 0.0f) {
      iter++;
    } else {
      iter = candidates.erase(iter);
    }
  }
  return candidates;
}