#ifndef LINEARCONTROLLER_HPP_
#define LINEARCONTROLLER_HPP

#include "Geometry/Point2d.hpp"

class LinearController
{
    public:
        LinearController(float kp, float k_feedforward,float maxVelocity);
        ~LinearController();

        Geometry::Point2d run(Geometry::Point2d err, Geometry::Point2d velocity);

        float _kp, _k_feedforward, _maxVelocity;

        Geometry::Point2d lastVelocity;
};
#endif