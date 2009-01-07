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

    ConfigFile::PidInfo pos = cfg.posPid;
    ConfigFile::PidInfo angle = cfg.anglePid;

    _xPID = new Pid(pos.p, pos.i, pos.d, pos.windup);
    _yPID = new Pid(pos.p, pos.i, pos.d, pos.windup);
    _anglePID = new Pid(angle.p, angle.i, angle.d, angle.windup);

    //_pathPlanner = 0;
}

Robot::~Robot()
{
    delete _xPID;
    delete _yPID;
    delete _anglePID;

    delete[] _motors;
}

void Robot::setSystemState(SystemState* state)
{
    _state = state;
}


void Robot::proc()
{
    float currAngle;
    Geometry::Point2d currPos;
    Geometry::Point2d currVel;
    Geometry::Point2d testDesiredPos;
    VelocityCmd velCmd;



    if(_state->self[_id].valid)
    {
        //printf("ID Please %d\n",_id);
        //TODO Send commands to motion via gameplay and set this flag there
        _state->self[_id].cmdValid = true;

        testDesiredPos.x = 1;
	testDesiredPos.y = 1;


        if(_state->self[_id].cmdValid)
        {
            currPos = _state->self[_id].pos;
            currVel = _state->self[_id].vel;
            currAngle = _state->self[_id].angle;

            //TODO position based control
            velCmd.vel.x = _xPID->run(testDesiredPos.x - currPos.x);;
            velCmd.vel.y = _yPID->run(testDesiredPos.y - currPos.y);

            printf("Pos in the x %f\n", currPos.x);
            printf("Pos in the y %f\n", currPos.y);

            velCmd.w = 0;

            genMotor(velCmd);

            _state->self[_id].cmdValid = false;
        }
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

    float max = 0;
    float mTemp[_axels.size()];
    float scale = 1;

    int i=0;
    Q_FOREACH(Geometry::Point2d axel, _axels)
    {
	//TODO - need to give the robots thier position info
	//axel.rotate(Point2d(0,0), _self.theta);

	//velocity in direction of wheel
	float v = axel.perpCW().dot(velCmd.vel);
	v += velCmd.w;
	//if greater than old max, set as max
	if (fabs(v) > max)
	{
		max = fabs(v);
	}

	mTemp[i++] = v;
    }

    const float mMax = velCmd.maxWheelSpeed;

    if (max > mMax)
    {
	scale = mMax/max;
    }

    for (unsigned int j=0; j<4; ++j)
    {
	const float req = mTemp[j] * scale;

	float change = req - _motors[j];

	const float mChange = 10.0;
	if (fabs(change) > mChange)
	{
		change = mChange/change * fabs(change);
	}

	_motors[j] += change;
        _state->radioCmd.robots[_id].motors[j] = (int8_t)(_motors[j]);
//         printf("Robot %d Motor %d = %d\n", _id,j, _state->radioCmd.robots[_id].motors[j]);
        //_comm.motor[i] = (int8_t)(_motors[i]);
    }
    _state->radioCmd.robots[_id].valid = true;
}

void Robot::clearPid()
{
	_xPID->clearWindup();
        _yPID->clearWindup();
	_anglePID->clearWindup();
}
