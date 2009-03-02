#include "Robot.hpp"

#include <Geometry/Point2d.hpp>
#include <Team.h>
#include "ConfigFile.hpp"
#include "Pid.hpp"
#include "framework/Module.hpp"

using namespace Geometry;

Robot::Robot(ConfigFile::RobotCfg cfg):
    _id(cfg.id)
{
    _axels = cfg.axels;
    _motors = new float[_axels.size()];

    //clear initial motor values and wheel velocities
    for (unsigned int i=0 ; i<4 ; ++i)
    {
        _motors[i] = 0;
        _lastWheelVel[i] = 0;
    }

    _maxAccel = cfg.maxAccel;
    _maxWheelVel = cfg.maxWheelVel;
    maxRobotVelocity = 250;

    _posController = new LinearController(cfg.posCntrlr.Kp,cfg.posCntrlr.Kv,maxRobotVelocity);
    _deadband = cfg.posCntrlr.deadband;

    rotationMatrix = new TransformMatrix();
}

Robot::~Robot()
{
    delete[] _motors;
}

void Robot::setSystemState(SystemState* state)
{
    _state = state;
}

void Robot::proc()
{
    if(_state->self[_id].valid)
    {
        float currAngle = _state->self[_id].angle;
        Geometry::Point2d currPos = _state->self[_id].pos;
        Geometry::Point2d currVel =_state->self[_id].vel;
        Geometry::Point2d desiredPos = _state->self[_id].cmdPos;
        Geometry::Point2d error = desiredPos - currPos;
        VelocityCmd velCmd;

        velCmd.vel = _posController->run(error,currVel);


//         velCmd.vel.x = 100*(error.x) + 0.0 * currVel.x;
//         velCmd.vel.y = 100*(error.y) + 0.0 * currVel.y;

//         printf("Curr Pos x %f y %f\n", currPos.x, currPos.y);
//         printf("Desired pos x %f y %f\n", desiredPos.x, desiredPos.y);
//         printf("Error x %f y %f \n", error.x, error.y);

//         printf("Velocity in Team Space x %f y %f \n", velCmd.vel.x, velCmd.vel.y);

//         velCmd.vel.x = 10;
//         velCmd.vel.y = 10;
        velCmd.w = 0;

        //Handle Deadband
        if(error.mag() < _deadband.mag())
        {
            velCmd.vel = Point2d(0,0);
        }

        //Rotate the velocity command from the team frame to the robot frame
        //The angle also needs to be converted
        rotationMatrix = new TransformMatrix(Point2d(0,0),(currAngle - 90.0), false, 1.0);
        velCmd.vel = rotationMatrix->transformDirection(velCmd.vel);

//         printf("CurrAngle %f \n",currAngle-90.0);
//         printf("Velocity in Robot Space x %f y %f \n", velCmd.vel.x, velCmd.vel.y);

        genMotor(velCmd);

        _state->self[_id].cmdValid = false;
    }
}


void Robot::genMotor(VelocityCmd velCmd)
{
    float maxGenWheelVel = 0;

    //clip the maximum spin velocity
    const float clip = 150;
    if (velCmd.w > clip)
    {
        velCmd.w = clip;
    }
    else if (velCmd.w < -clip)
    {
        velCmd.w = -clip;
    }

    int j =0;
    Q_FOREACH(Geometry::Point2d axel, _axels)
    {
        _motors[j] = axel.perpCW().dot(velCmd.vel);

//         v += velCmd.w;

        //Limit wheel acceleration
        if(abs(_lastWheelVel[j] - _motors[j]) > _maxAccel)
        {
            //Set v such that we achieve the max allowable acceleration
            _motors[j] = _lastWheelVel[j] - _maxAccel;
        }

        if(abs(_motors[j] > maxGenWheelVel))
        {
            maxGenWheelVel = _motors[j];
//             printf("max wheel speed %f\n",_motors[j]);
        }

        _lastWheelVel[j] = _motors[j];
        j++;
    }
    //One of the wheels has gone into saturation
    if(maxGenWheelVel > _maxWheelVel)
    {
//         printf("Saturation\n");
        //Scale it and the other wheels to achieveable velocities
        for(int i = 0; i<4; i++)
        {
            _motors[i] *= _maxWheelVel / maxGenWheelVel;
        }
    }

    for(int j = 0; j<4; j++)
    {
        _state->radioCmd.robots[_id].motors[j] = (int8_t)_motors[j];
    }
//     printf("Wheel Speeds [0] %f [1] %f [2] %f [3] %f\n",_motors[0], _motors[1], _motors[2], _motors[3]);
    _state->radioCmd.robots[_id].valid = true;
}

void Robot::clearPid()
{
}
