#include "LinearController.hpp"
#include <stdio.h>
#include <stdlib.h>

LinearController::LinearController(float kp, float k_feedforward, float maxVelocity)
{
    _kp = kp;
    _k_feedforward = k_feedforward;
    _maxVelocity = maxVelocity;

}

LinearController::~LinearController()
{

}

Geometry::Point2d LinearController::run(Geometry::Point2d err, Geometry::Point2d velocity)
{
    Geometry::Point2d output;
    Geometry::Point2d accel;

    output = (err * _kp) + (velocity * _k_feedforward);

//     printf("LinearController %f %f\n" ,_kp, _k_feedforward);

//     accel = (output - lastVelocity);

//     output.x += (accel.y * 0.5);
//     output.y += (accel.x * 0.5);

    if(output.x > output.y)
    {
//         output.x -= ((accel.x - accel.y) * (0.6 * velocity.x));
//         output.y += ((accel.x - accel.y) * 1);

        if(output.x > _maxVelocity)
        {
            printf("Saturation\n");
            output.x = _maxVelocity;
            output.y *= _maxVelocity / output.x;
        }
    }
    else
    {
//         output.x += ((accel.y - accel.x) * 1);
//         output.y -= ((accel.y - accel.x) * (0.6 * velocity.y));

        if(output.y > _maxVelocity)
        {
            printf("Saturation\n");
            output.y = _maxVelocity;
            output.x *= _maxVelocity / output.y;
        }
    }

    return output;
}