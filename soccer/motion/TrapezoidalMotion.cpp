
#include "TrapezoidalMotion.hpp"
#include <math.h>


bool TrapezoidalMotion(
	float pathLength,
	float maxSpeed,
	float maxAcc,
	float timeIntoLap,
	float currentSpeed,
	float finalSpeed,
	float &posOut,
	float &speedOut)
{
	//	when we're speeding up and slowing down - the sides of the trapezoid
	float rampTime = maxSpeed / maxAcc;
	float rampDist = 0.5 * maxAcc * powf(rampTime, 2.0);	//	Sf = 1/2*a*t^2

	//	when we're going at max speed
	float distAtMaxSpeed = (pathLength - 2.0 * rampDist);
	float timeAtMaxSpeed = distAtMaxSpeed / maxSpeed;

	if (timeIntoLap < rampTime) {	//	we're speeding up
		posOut = 0.5 * maxAcc * timeIntoLap * timeIntoLap;
		speedOut = maxAcc * timeIntoLap;
	} else if (timeIntoLap < (rampTime + timeAtMaxSpeed)) {	//	at plateau, going max speed
		posOut = rampDist + maxSpeed * (timeIntoLap - rampTime);
		speedOut = maxSpeed;
	} else if (timeIntoLap < timeAtMaxSpeed + rampTime*2) {	//	we're slowing down
		float deccelTime = timeIntoLap - (rampTime + timeAtMaxSpeed);
		posOut = rampDist + distAtMaxSpeed + 
					maxSpeed * deccelTime - 0.5 * maxAcc * deccelTime * deccelTime;
		speedOut = maxSpeed - deccelTime * maxAcc;
	} else {
		//	restart for another lap
		return false;
	}

	return true;
}
