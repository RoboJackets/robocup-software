#include "Robot.hpp"

#include <Geometry/Point2d.hpp>

using namespace Geometry;
using namespace Packet;

Robot::Robot(ConfigFile::RobotCfg cfg, 
		Packet::VisionData& vd, 
		Packet::CommData::Robot& cd,
		Packet::LogMotion::Robot& ld) :
	_id(cfg.id), _vision(vd), _self(vd.self[cfg.id]), _comm(cd), _log(ld)
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
	
	_posPID = new Pid(pos.p, pos.i, pos.d, pos.windup);	
	_anglePID = new Pid(angle.p, angle.i, angle.d, angle.windup);
	
	_pathPlanner = 0;
}

Robot::~Robot()
{
	delete _posPID;
	delete _anglePID;
	
	delete[] _motors;
}

void Robot::proc(const Packet::MotionCmd::Robot& cmd)
{
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
		axel.rotate(Point2d(0,0), _self.theta);
		
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

	for (unsigned int i=0; i<4; ++i)
	{
		const float req = mTemp[i] * scale;
		
		float change = req - _motors[i];
		
		const float mChange = 10.0;
		if (fabs(change) > mChange)
		{
			change = mChange/change * fabs(change);
		}
		
		_motors[i] += change;
		
		_comm.motor[i] = (int8_t)(_motors[i]);
	}
}

void Robot::clearPid()
{
	_posPID->clearWindup();
	_anglePID->clearWindup();
}
