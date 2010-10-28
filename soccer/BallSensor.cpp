#include "BallSensor.hpp"

#include <Constants.hpp>
#include <Utils.hpp>

#include <framework/SystemState.hpp>
#include <gameplay/GameplayModule.hpp>

#include <boost/foreach.hpp>

using namespace boost;

const int Bad_Sensor_Threshold = 15;

void updateStatusOfBallSensors(SystemState * _state)
{
    const Ball &ball = _state->ball;

    if(!ball.valid)
    {
        return;
    }

    BOOST_FOREACH(OurRobot * robot, _state->self)
    {
        if(robot->hasBall)
        {
            if((!robot->pos.nearPoint(ball.pos, Robot_Radius + Ball_Radius + 15)) 
                    && (robot->sensorConfidence < Bad_Sensor_Threshold))
            {
                robot->sensorConfidence++;
            }

            if(robot->sensorConfidence >= Bad_Sensor_Threshold)
            {
                robot->hasBall = false;
            }
        }
        else
        {
            robot->sensorConfidence = 0;
        }
    }
}
