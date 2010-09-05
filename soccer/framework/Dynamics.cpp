#include "Dynamics.hpp"
#include <framework/RobotConfig.hpp>

#include <math.h>

using namespace std;
using namespace Planning;

Dynamics::Dynamics(OurRobot *robot):
	_self(robot)
{
}

Dynamics::DynamicsInfo Dynamics::info(const float angle, const float w) const
{
	Dynamics::DynamicsInfo info;
	
	//given and angle and rotational speed
	//compute the linear capabilities of the system
	
	const float vMax0 = _self->config->motion.deg0.velocity;
	const float vMax45 = _self->config->motion.deg45.velocity;
	
	const float aMax0 = _self->config->motion.deg0.acceleration;
	const float aMax45 = _self->config->motion.deg45.acceleration;
	
	const float dMax0 = _self->config->motion.deg0.deceleration;
	const float dMax45 = _self->config->motion.deg45.deceleration;
	
	float clipped = fabs(angle);
	
	//bring the angle within range of [0...45]
	if (clipped > 45)
	{
		clipped -= (int(clipped/45.0f) * 45);
	}
	

	// linear interpolation of the values
	//TODO should use sin or something because it is a circle
	//not a fixed ratio
	float percent = clipped/45.0f;
	
	const float vDiff = vMax0 - vMax45;
	const float aDiff = aMax0 - aMax45;
	const float dDiff = dMax0 - dMax45;
	
	info.velocity = vMax0 - vDiff * percent;
	info.acceleration = aMax0 - aDiff * percent;
	info.deceleration = dMax0 - dDiff * percent;
	
	//1 - percent of rotation out of max
	float wPercent = 1;
	
	if (_self->config->motion.rotation.velocity != 0)
	{
		wPercent -= fabs(w/_self->config->motion.rotation.velocity);
	}
	
	if (wPercent < 0)
	{
		wPercent = 0;
	}
	
	info.velocity *= wPercent;
	info.acceleration *= wPercent;
	info.deceleration *= wPercent;
	
	return info;
}

float Dynamics::travelTime(const float length) const
{
	if (_self->config->motion.deg0.velocity == 0)
	{
		return 0;
	}
	
	//for now assume the max velocity
	return length/_self->config->motion.deg0.velocity;
}
