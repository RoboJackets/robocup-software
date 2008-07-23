#include "Pid.hpp"

#include <cstring>
#include <stdio.h>

Pid::Pid(float p, float i, float d, unsigned int windup) :
	_windup(windup)
{
	_windupLoc = 0;
	_errSum = 0;

	_oldErr = new float[_windup];
	memset(_oldErr, 0, sizeof(float)*_windup);

	_lastErr = 0;

	kp = p;
	ki = i;
	kd = d;
}

Pid::~Pid()
{
	delete[] _oldErr;
}

float Pid::run(float err)
{
	float dErr = err - _lastErr;
	_lastErr = err;

	_errSum += err;
	_errSum -= _oldErr[_windupLoc];
	_oldErr[_windupLoc] = err;

	if (++_windupLoc > _windup)
	{
		_windupLoc = 0;
	}

	return err * kp + _errSum * ki + dErr * kd;
}

void Pid::clearWindup()
{
	for (unsigned int i=0 ; i<_windup ; ++i)
	{
		_oldErr[i] = 0;
	}
}
