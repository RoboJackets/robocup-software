#pragma once
#include <vector>

class Pid
{
public:
	Pid(float p = 0, float i = 0, float d = 0, unsigned int windup = 0);


	unsigned int windup() const { return _windup; }

	/** Runs the pid loop and returns the error */
	float run(const float err);

	void setWindup(unsigned int w);

	/** clear any windup */
	void clearWindup();

	float kp, ki, kd;

private:
	Pid(Pid&);
	Pid& operator &= (Pid&);

	/** amount to sum up */
	unsigned int _windup;

	unsigned int _windupLoc;

	float _errSum;

	float _lastErr;

	std::vector<float> _oldErr;
};
