#include "RobotFilter.hpp"
#include <Utils.hpp>
#include <iostream>

using namespace Geometry2d;

// How long to coast a robot's position when it isn't visible
static const float Coast_Time = 0.8;
static const float Velocity_Alpha = 0.2;
static const float Min_Frame_Time = 0.014;

RobotFilter::RobotFilter()
{
	_camera = -1;
}

void RobotFilter::update(const RobotObservation* obs)
{
	if (obs->source < 0 || obs->source >= Num_Cameras)
	{
		// Not from a camera?
		return;
	}
	
	int s = obs->source;
	double dtime = (obs->time - _estimate[s].time) / 1000000.0f;
	bool reset = _estimate[s].time == 0 || (dtime > Coast_Time);
	
	if (reset || dtime < Min_Frame_Time)
	{
		_estimate[s].vel = Point();
		_estimate[s].angleVel = 0;
	} else {
		Point newVel = (obs->pos - _estimate[s].pos) / dtime;
		_estimate[s].vel = newVel * Velocity_Alpha + _estimate[s].vel * (1.0f - Velocity_Alpha);
		
		double newW = fixAngleRadians(obs->angle - _estimate[s].angle) / dtime;
		_estimate[s].angleVel = newW * Velocity_Alpha + _estimate[s].angleVel * (1.0f - Velocity_Alpha);
	}
	_estimate[s].pos = obs->pos;
	_estimate[s].angle = obs->angle;
	_estimate[s].visible = true;
	_estimate[s].time = obs->time;
	_estimate[s].visionFrame = obs->frameNumber;
}

void RobotFilter::predict(Time time, Robot* robot)
{
	int bestSource = -1;
	double bestDTime = 0;
	for (int s = 0; s < Num_Cameras; ++s)
	{
		double dtime = (time - _estimate[s].time) / 1000000.0f;
		if (_estimate[s].visible && (bestSource < 0 || dtime < bestDTime))
		{
			bestSource = s;
			bestDTime = dtime;
		}
	}
	
	if (bestSource < 0)
	{
		robot->visible = false;
		return;
	}
	
	robot->pos = _estimate[bestSource].pos + _estimate[bestSource].vel * bestDTime;
	robot->vel = _estimate[bestSource].vel;
	robot->angle = fixAngleRadians(_estimate[bestSource].angle + _estimate[bestSource].angleVel * bestDTime);
	robot->angleVel = _estimate[bestSource].angleVel;
	robot->visible = _estimate[bestSource].visible && bestDTime < Coast_Time;
}
