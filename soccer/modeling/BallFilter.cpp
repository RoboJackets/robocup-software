#include "BallFilter.hpp"
#include "BallTracker.hpp"

#include <SystemState.hpp>

#include <stdio.h>

using namespace Geometry2d;

static const float Velocity_Alpha = 0.2;

BallFilter::BallFilter()
{
}

void BallFilter::update(const BallObservation* obs)
{
	if (_estimate.time)
	{
		float dtime = (obs->time - _estimate.time) / 1000000.0f;
		Point newVel = (obs->pos - _estimate.pos) / dtime;
		_estimate.vel = newVel * Velocity_Alpha + _estimate.vel * (1.0f - Velocity_Alpha);
		_estimate.valid = true;
	} else {
		_estimate.vel = Point();
	}
	
	_estimate.pos = obs->pos;
	_estimate.time = obs->time;
}

void BallFilter::predict(Time time, Ball *out, float *velocityUncertainty)
{
	if (velocityUncertainty)
	{
		*velocityUncertainty = 2 + _estimate.vel.mag() * 0.5;
	}

	if (out)
	{
		out->pos = _estimate.pos + _estimate.vel * (time - _estimate.time) / 1000000.0f;
		out->vel = _estimate.vel;
		out->time = time;
		out->valid = true;
	}
}
