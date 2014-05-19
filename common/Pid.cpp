#include "Pid.hpp"

#include <cstring>
#include <stdio.h>

Pid::Pid(float p, float i, float d, unsigned int windup) :
	_windup(windup)
{
	_windupLoc = 0;
	_errSum = 0;
	_oldErr = 0;
	
	_lastErr = 0;

	kp = p;
	ki = i;
	kd = d;
	
	setWindup(_windup);
}

Pid::~Pid()
{
	if (_oldErr)
	{
		delete[] _oldErr;
	}
}

void Pid::setWindup(unsigned int w)
{
	if (w > 0)
	{
		_windup = w;
		_oldErr = new float[_windup];
		memset(_oldErr, 0, sizeof(float)*_windup);
	} else {
		if (_oldErr) delete _oldErr;
		_oldErr = nullptr;
	}
}

float Pid::run(const float err)
{
	float dErr = err - _lastErr;
	_lastErr = err;

	_errSum += err;
	
	if (_oldErr)
	{
		_errSum -= _oldErr[_windupLoc];
		_oldErr[_windupLoc] = err;
	
		if (++_windupLoc > _windup)
		{
			_windupLoc = 0;
		}
	}
	
	return err * kp + _errSum * ki + dErr * kd;
}

void Pid::clearWindup()
{
	if (_oldErr)
	{
		for (unsigned int i=0 ; i<_windup ; ++i)
		{
			_oldErr[i] = 0;
		}
	}
}
