#include "Robot.hpp"

#include <Geometry/Point2d.hpp>
#include <Team.h>
#include <config/ConfigFile.hpp>
#include <boost/foreach.hpp>

#include "Pid.hpp"
#include "framework/Module.hpp"

using namespace Geometry;

Robot::Robot(ConfigFile::RobotCfg cfg, unsigned int id):
    _id(id)
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

    _posController = new LinearController(cfg.posCntrlr.Kp, cfg.posCntrlr.Kd, cfg.maxRobotVel);
    _deadband = cfg.posCntrlr.deadband;

    rotationMatrix = new TransformMatrix();\

    _pathPlanner = new PathPlanner();
}

Robot::~Robot()
{
    delete[] _motors;
}

void Robot::setKp(double value)
{
    _posController->setKp(value);
}

void Robot::setKd(double value)
{
    _posController->setKd(value);
}

void Robot::proc()
{
    if(_state->self[_id].valid)
    {
        _goalPos = _state->self[_id].cmd.goal;
        _currPos = _state->self[_id].pos;

        if(!_path.waypoints.empty())
        {
            _path.waypoints.clear();
        }

        _pathPlanner->setState(_state);
        _path = _pathPlanner->plan(_currPos, _goalPos);

        genVelocity();
    }
}

void Robot::genVelocity()
{
    float currAngle = _state->self[_id].angle;
    Geometry::Point2d currPos = _state->self[_id].pos;
    Geometry::Point2d feedforwardVelocity = _state->self[_id].cmd.v_ff;
    Geometry::Point2d error = _state->self[_id].cmd.pos - currPos;
    VelocityCmd velCmd;

    velCmd.vel = _posController->run(error,feedforwardVelocity);
    velCmd.w = 0;
//         printf("Curr Pos x %f y %f\n", currPos.x, currPos.y);
//         printf("Desired pos x %f y %f\n", desiredPos.x, desiredPos.y);
//         printf("Error x %f y %f \n", error.x, error.y);
//         printf("Velocity in Team Space x %f y %f \n", velCmd.vel.x, velCmd.vel.y);

    //Deadband
    if(error.mag() < _deadband.mag())
    {
        velCmd.vel = Point2d(0,0);
    }

    _state->self[_id].vel = velCmd.vel;

    //Rotate the velocity command from the team frame to the robot frame
    //The angle also needs to be converted
    rotationMatrix = new TransformMatrix(Point2d(0,0),(currAngle - 90.0), false, 1.0);
    velCmd.vel = rotationMatrix->transformDirection(velCmd.vel);

//         printf("Velocity in Robot Space x %f y %f \n", velCmd.vel.x, velCmd.vel.y);

    genMotor(velCmd);
}

void Robot::genMotor(VelocityCmd velCmd)
{
    float maxGenWheelVel = 0;
    unsigned int shellId = _state->self[_id].shell;

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
    BOOST_FOREACH(Geometry::Point2d axel, _axels)
    {
        _motors[j] = axel.perpCW().dot(velCmd.vel);

        //Limit wheel acceleration
//         if(abs(_lastWheelVel[j] - _motors[j]) > _maxAccel)
//         {
//             //Set v such that we achieve the max allowable acceleration
//             _motors[j] = _lastWheelVel[j] - _maxAccel;
//         }

        if(abs(_motors[j] > maxGenWheelVel))
        {
            maxGenWheelVel = _motors[j];
        }

        _lastWheelVel[j] = _motors[j];
        j++;
    }
    //One of the wheels has gone into saturation
    if(maxGenWheelVel > _maxWheelVel)
    {
        printf("Wheel Speeds before [0] %f [1] %f [2] %f [3] %f\n",_motors[0], _motors[1], _motors[2], _motors[3]);
//         printf("Wheel saturation\n");
        //Scale it and the other wheels to achieveable velocities
        for(int i = 0; i<4; i++)
        {
            _motors[i] *= _maxWheelVel / maxGenWheelVel;
        }
        printf("Wheel Speeds after [0] %f [1] %f [2] %f [3] %f\n",_motors[0], _motors[1], _motors[2], _motors[3]);
    }

    _state->radioCmd.robots[_id].board_id = shellId;

    for(int j = 0; j<4; j++)
    {
        _state->radioCmd.robots[_id].motors[j] = (int8_t)_motors[j];
    }
//     printf("Wheel Speeds after [0] %f [1] %f [2] %f [3] %f\n",_motors[0], _motors[1], _motors[2], _motors[3]);
    _state->radioCmd.robots[_id].valid = true;
}

