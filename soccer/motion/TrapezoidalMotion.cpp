
#include "TrapezoidalMotion.hpp"
#include <math.h>


bool TrapezoidalMotion(
	float pathLength,
	float maxSpeed,
	float maxAcc,
	float timeIntoLap,
	float startSpeed,	//	USEME
	float finalSpeed,	//	USEME
	float &posOut,
	float &speedOut)
{
	//	when we're speeding up and slowing down - the sides of the trapezoid
	float rampDeltaSpeed = maxSpeed - startSpeed;
	float downDeltaSpeed = maxSpeed - finalSpeed;
	float rampTime = rampDeltaSpeed / maxAcc;
	float downTime = downDeltaSpeed / maxAcc;
	float aveRampSpeed = rampDeltaSpeed / 2.0 + startSpeed;
	float aveDownSpeed = downDeltaSpeed / 2.0 + finalSpeed;
	float rampDist = (rampTime * aveRampSpeed);
	float downDist = (downTime * aveDownSpeed);
	if(rampDist + downDist >= pathLength) {
		/*d = (maxSpeed + startSpeed) / 2.0 * rampTime + (maxSpeed + finalSpeed) / 2.0 * downTime;
		d = (maxSpeed + startSpeed) / 2.0 *(maxSpeed - startSpeed) / maxAcc +
				(maxSpeed + finalSpeed) / 2.0 * (maxSpeed - finalSpeed) / maxAcc;
		d = (maxSpeed^2 - startSpeed^2) / (2.0 * maxAcc) + (maxSpeed^2 - finalSpeed^2) / (2.0 * maxAcc);
		(2.0 * maxAcc) * d = maxSpeed^2 - startSpeed^2 + maxSpeed^2 - finalSpeed^2
		maxSpeed^2 = maxAcc * d  - (startSpeed^2 + finalSpeed^2) / 2*/
		maxSpeed = sqrt(maxAcc * pathLength - (startSpeed * startSpeed + finalSpeed * finalSpeed) / 2);
		rampTime = (maxSpeed - startSpeed) / maxAcc;
		downTime = (maxSpeed - finalSpeed) / maxAcc;

		rampTime = sqrt(pathLength / maxAcc);
		maxSpeed = rampTime * maxAcc;
		if (timeIntoLap < rampTime) {	//	we're speeding up
			posOut = startSpeed * timeIntoLap + 0.5 * maxAcc * timeIntoLap * timeIntoLap;
			speedOut = startSpeed + maxAcc * timeIntoLap;
		} else if (timeIntoLap <= rampTime * 2.0) {
			rampDist = ((maxSpeed + startSpeed) / 2) * rampTime;
			float deccelTime = timeIntoLap - (rampTime);
			posOut = rampDist +	maxSpeed * deccelTime - 0.5 * maxAcc * deccelTime * deccelTime;
			speedOut = maxSpeed - deccelTime * maxAcc;
		} else {
			posOut = pathLength;
			speedOut = 0;
			return false;
		}
		return true;
	} else {
		//	when we're going at max speed
		float distAtMaxSpeed = (pathLength - rampDist - downDist);
		float timeAtMaxSpeed = distAtMaxSpeed / maxSpeed;

		if (timeIntoLap < rampTime) {	//	we're speeding up
			posOut = startSpeed * timeIntoLap + 0.5 * maxAcc * timeIntoLap * timeIntoLap;
			speedOut = startSpeed + maxAcc * timeIntoLap;
		} else if (timeIntoLap <= (rampTime + timeAtMaxSpeed)) {	//	at plateau, going max speed
			posOut = rampDist + maxSpeed * (timeIntoLap - rampTime);
			speedOut = maxSpeed;
		} else if (timeIntoLap <= timeAtMaxSpeed + rampTime + downTime) {	//	we're slowing down
			float deccelTime = timeIntoLap - (rampTime + timeAtMaxSpeed);
			posOut = rampDist + distAtMaxSpeed + 
						maxSpeed * deccelTime - 0.5 * maxAcc * deccelTime * deccelTime;
			speedOut = maxSpeed - deccelTime * maxAcc;
		} else {
			posOut = pathLength;
			speedOut = 0;
			return false;
		}

		return true;
	}
}
