#include "LinearController.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

LinearController::LinearController(float kp, float kd, float maxVelocity )
{
    _kp = kp;
    _maxVelocity = maxVelocity;
    _kd = kd;

}

LinearController::~LinearController()
{

}

Geometry::Point2d LinearController::run(Geometry::Point2d err, Geometry::Point2d velocity)
{
    Geometry::Point2d output;
    Geometry::Point2d dErr = err - lastErr;
    lastErr = err;

    output = (err * _kp) + (dErr * _kd) + velocity;

    if(output.mag() > _maxVelocity)
    {
        float angle = output.angle();
        output.x = _maxVelocity*cos(angle);
        output.y = _maxVelocity*sin(angle);
    }

    return output;
}

void LinearController::setKp(double kp)
{
    _kp = (float) kp;
}

void LinearController::setKd(double kd)
{
    _kd = (float) kd;
}
