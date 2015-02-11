#include "Pid.hpp"

#include <cstring>
#include <stdio.h>
#include <iostream>
#include <math.h>

using namespace std;


Pid::Pid(float p, float i, float d, unsigned int windup)
{
	_windupLoc = 0;
	_errSum = 0;
	_oldErr = 0;
	
	_lastErr = 0;

	kp = p;
	ki = i;
	kd = d;
	
	_windup = 0;
	setWindup(windup);
}

Pid::~Pid()
{
	if (_oldErr)
	{
		delete[] _oldErr;
		_oldErr = nullptr;
	}
}

void Pid::setWindup(unsigned int w)
{
	if (w != _windup) 
	{
		if (_oldErr) 
			delete[] _oldErr;

		if (w > 0)
		{
			if (w != _windup) {
				_windup = w;
				_oldErr = new float[_windup]{};
				_errSum=0;
			}
		}
	}
}

float Pid::run(const float err)
{
	if (isnan(err))
	{
		return 0;
	}
	float dErr = err - _lastErr;
	_lastErr = err;

	_errSum += err;
	
	if (_oldErr)
	{
		_errSum -= _oldErr[_windupLoc];
		_oldErr[_windupLoc] = err;
	
		_windupLoc = (_windupLoc + 1) % _windup;
	}

	return (err * kp) + (_errSum * ki) + (dErr * kd);
}

void Pid::clearWindup()
{
	if (_oldErr)
	{
		memset(_oldErr, 0, sizeof(float)*_windup);
	}
}
