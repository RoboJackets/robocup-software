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

    //clear initial motor values
    for (unsigned int i=0 ; i<4 ; ++i)
    {
	    _motors[i] = 0;
    }

    _maxAccel = cfg.maxAccel;
    _maxWheelVel = cfg.maxWheelVel;

    _Kp = cfg.posCntrlr.Kp;
    _Kv = cfg.posCntrlr.Kv;
    /*
    ConfigFile::PidInfo pos = cfg.posPid;
    ConfigFile::PidInfo angle = cfg.anglePid;

    _xPID = new Pid(pos.p, pos.i, pos.d, pos.windup);
    _yPID = new Pid(pos.p, pos.i, pos.d, pos.windup);
    _anglePID = new Pid(angle.p, angle.i, angle.d, angle.windup);
    */
    rotationMatrix = new TransformMatrix();

    //_pathPlanner = 0;
}

Robot::~Robot()
{
//     delete _xPID;
//     delete _yPID;
//     delete _anglePID;

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
        Geometry::Point2d currVel = _state->self[_id].vel;
        Geometry::Point2d desiredPos = Point2d(1.0,1.0); //_state->self[_id].cmdPos;
        Geometry::Point2d error = currPos - desiredPos;
        VelocityCmd velCmd;
        int kp = 80;
        int k_feedforward = 10;

        //TODO handle deadband
        velCmd.vel.x = kp*(error.x) + k_feedforward * currVel.x;
//         velCmd.vel.y = kp*(error.y) + k_feedforward * currVel.y;

        printf("Curr Pos x %f y %f\n", currPos.x, currPos.y);
        printf("Desired pos x %f y %f\n", desiredPos.x, desiredPos.y);
        printf("Error x %f y %f \n", error.x, error.y);

//         velCmd.vel.x = 100;
        velCmd.vel.y = 0;
        velCmd.w = 0;

        //Rotate the velocity command from the team frame to the robot frame
        rotationMatrix = new TransformMatrix(Point2d(0,0),currAngle, false, 1.0);
        velCmd.vel = rotationMatrix->transformDirection(velCmd.vel);

        genMotor(velCmd);

        _state->self[_id].cmdValid = false;
    }
    /*
    if (_self.valid)
    {
        _comm.valid = true;
        _log.valid = true;

        //output velocity command for motor speed generation
        VelocityCmd velCmd;

        _log.currPos = _self.pos;

        const float currAngle = _self.theta;

        //change in x,y for angle calculation
        Point2d dxdy = cmd.face - _self.pos;

        //destination point for path planner
        Point2d dest = cmd.pos;

        if (cmd.valid)
        {
            if (_id)
            {
                _pathPlanner->addNoZone(Circle2d(0,0, .7));
            }

            //avoid zone
            if (cmd.avoid)
            {
                _pathPlanner->addNoZone(cmd.avoidZone);
            }

            //select style of motion
            switch (cmd.style)
            {
                case MotionCmd::Fast:
                    velCmd.maxWheelSpeed = 90;
                    break;
                case MotionCmd::Accurate:
                    velCmd.maxWheelSpeed = 50;
                    break;
            }

            float tAngle = atan2(dxdy.y, dxdy.x) * 180.0 / M_PI;
            float err = currAngle - tAngle;

            //clip error to +/-180 deg
            if (err > 180)
            {
                err -= 360.0;
            }
            else if (err < -180)
            {
                err += 360.0;
            }

            velCmd.w = _anglePID->run(err);

            //passthrough data
            _comm.kick = cmd.kick;
            _comm.roller = cmd.roller;
        }
        else
        {
            //no command, but we have vision
            //plan path to current location
            //this gets us out of noZones if there are any
            dest = _self.pos;
            velCmd.w = 0;
            velCmd.maxWheelSpeed = 60;

            //turn off
            _comm.kick = 0;
            _comm.roller = 0;
        }

        PathPlanner::PPOut ppout = _pathPlanner->plan(_id, dest);

        _log.pdir = ppout.direction;
        _log.distRemaining = ppout.distance;
        _log.destPos = dest;

        ppout.direction *= _posPID->run(ppout.distance);
        velCmd.vel = ppout.direction;

        //if spinning, clear linear vel
        if (cmd.valid && cmd.spin)
        {
            velCmd.vel = Point2d();
            velCmd.w = 100;
        }

        //generate motor speeds and populate outgoing data
        genMotor(velCmd);
    }
    else
    {
        //turn everything off if there is no vision
        _comm.valid = false;
    }
    */
}


void Robot::genMotor(VelocityCmd velCmd)
{
    static int8_t lastWheelVel[4];

    for(int k = 0; k<4; k++)
    {
        lastWheelVel[k] = 0;
    }

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
        int8_t v = (int8_t)axel.perpCW().dot(velCmd.vel);

        v += velCmd.w;

        //Limit wheel acceleration
        if(abs(lastWheelVel[j] - v) > _maxAccel)
        {
            //Set v such that we achieve the max allowable acceleration
            v = lastWheelVel[j] - _maxAccel;
        }

        //Saturate on max velocity
        if(v > _maxWheelVel)
        {
            v = _maxWheelVel;
        }
        else if(v < -_maxWheelVel)
        {
            v = -_maxWheelVel;
        }

        _state->radioCmd.robots[_id].motors[j++] = v;
        lastWheelVel[j] = v;
    }
    _state->radioCmd.robots[_id].valid = true;
}

void Robot::clearPid()
{
    _xPID->clearWindup();
    _yPID->clearWindup();
    _anglePID->clearWindup();
}
