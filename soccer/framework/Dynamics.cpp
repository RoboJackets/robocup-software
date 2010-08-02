#include "Dynamics.hpp"
#include <iostream>

using namespace std;
using namespace Planning;

Dynamics::Dynamics()
{
	
}

void Dynamics::setConfig(ConfigFile::Robot::Motion cfg)
{
	//cout << "Dynamics::setConfig(ConfigFile::Robot::Motion cfg)" << endl;
	_deg0 = cfg.deg0;
	_deg45 = cfg.deg45;
	_rotation = cfg.rotation;
}

Dynamics::DynamicsInfo Dynamics::info(const float angle, const float w) const
{
	Dynamics::DynamicsInfo info;
	
	//given and angle and rotational speed
	//compute the linear capabilities of the system
	
	const float vMax0 = _deg0.velocity;
	const float vMax45 = _deg45.velocity;
	
	const float aMax0 = _deg0.acceleration;
	const float aMax45 = _deg45.acceleration;
	
	const float dMax0 = _deg0.deceleration;
	const float dMax45 = _deg45.deceleration;
	
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
	
	if (_rotation.velocity != 0)
	{
		wPercent -= fabs(w/_rotation.velocity);
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
	if (_deg0.velocity == 0)
	{
		return 0;
	}
	
	//for now assume the max velocity
	return length/_deg0.velocity;
}
