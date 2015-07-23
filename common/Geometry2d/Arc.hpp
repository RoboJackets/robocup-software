//
// Created by matt on 7/19/15.
//

#pragma once

#include "Point.hpp"
#include "Line.hpp"
#include "Segment.hpp"

namespace Geometry2d {
  class Arc {
  public:
    Arc() {
      _radius = -1;
      _start_angle = 0;
      _end_angle = 0;
    }

    Arc(Point center, float radius, float start, float end) {
      _center = center;
      _radius = radius;
      _start_angle = start;
      _end_angle = end;
    }

    Point center() const {
      return _center;
    }

    void setCenter(Point center) {
      _center = center;
    }

    float radius() const {
      return _radius;
    }

    void setRadius(float radius) {
      _radius = radius;
    }

    float start() const {
      return _start_angle;
    }

    void setStart(float start) {
      _start_angle = start;
    }

    float end() const {
      return _end_angle;
    }

    void setEnd(float end) {
      _end_angle = end;
    }

    float radius_sq() const {
      return _radius * _radius;
    }

    std::vector<Point> intersects(const Line& line) const;

    std::vector<Point> intersects(const Segment& segment) const;

  private:
    Point _center;
    float _radius;
    float _start_angle;
    float _end_angle;
  };
}
