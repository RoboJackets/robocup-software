#ifndef LINEARCONTROLLER_HPP_
#define LINEARCONTROLLER_HPP

#include "Geometry/Point2d.hpp"

class LinearController
{
    public:
        LinearController(float kp, float kd, float maxVelocity);
        ~LinearController();

        Geometry::Point2d run(Geometry::Point2d err, Geometry::Point2d velocity);

        float _kp, _kd, _maxVelocity;

        Geometry::Point2d lastVelocity;
        Geometry::Point2d lastErr;
};
#endif