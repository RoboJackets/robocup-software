#include "RobotFilter.hpp"
#include <Utils.hpp>

using namespace Utils;
using namespace Geometry2d;

// How long to coast a robot's position when it isn't visible
static const float Coast_Time = 0.8;

static const float Velocity_Alpha = 0.2;

RobotFilter::RobotFilter()
{
}

void RobotFilter::update(const RobotObservation* obs)
{
	float dtime = (obs->time - _estimate.time) / 1000000.0f;
	bool reset = _estimate.time == 0 || (dtime > Coast_Time);
	
	if (reset || dtime == 0)
	{
		_estimate.vel = Point();
		_estimate.angleVel = 0;
	} else {
		Point newVel = (obs->pos - _estimate.pos) / dtime / 0.8;
		_estimate.vel = newVel * Velocity_Alpha + _estimate.vel * (1.0f - Velocity_Alpha);
		
		float newW = fixAngleDegrees(obs->angle - _estimate.angle) / dtime / 1.15;
		_estimate.angleVel = newW * Velocity_Alpha + _estimate.angleVel * (1.0f - Velocity_Alpha);
	}
	_estimate.pos = obs->pos;
	_estimate.angle = obs->angle;
	_estimate.visible = true;
	_estimate.time = obs->time;
	_estimate.visionFrame = obs->frameNumber;
}

void RobotFilter::predict(uint64_t time, Robot* robot)
{
#if 0
	float dtime = (time - _estimate.time) / 1000000.0f;
	robot->pos = _estimate.pos;
	robot->vel = _estimate.vel;
	robot->angle = _estimate.angle;
	robot->angleVel = _estimate.angleVel;
	robot->visible = _estimate.visible && dtime < Coast_Time;
	robot->visionFrame = _estimate.visionFrame;
#else
	float dtime = (time - _estimate.time) / 1000000.0f;
	robot->pos = _estimate.pos + _estimate.vel * dtime;
	robot->vel = _estimate.vel;
	robot->angle = fixAngleDegrees(_estimate.angle + _estimate.angleVel * dtime);
	robot->angleVel = _estimate.angleVel;
	robot->visible = _estimate.visible && dtime < Coast_Time;
#endif
}
