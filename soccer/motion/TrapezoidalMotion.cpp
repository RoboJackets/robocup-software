#include "TrapezoidalMotion.hpp"
#include <math.h>


bool TrapezoidalMotion(
	float pathLength,
	float maxSpeed,
	float maxAcc,
	float timeIntoLap,
	float startSpeed,
	float finalSpeed,
	float &posOut,
	float &speedOut)
{
	//	begin by assuming that there's enough time to get up to full speed
	//	we do this by calculating the full ramp-up and ramp-down, then seeing
	//	if the distance travelled is too great.  If it's gone too far, this is the "triangle case"

	startSpeed = fmin(startSpeed, maxSpeed);
	finalSpeed = fmin(finalSpeed, maxSpeed);
	float rampUpTime = (maxSpeed - startSpeed) / maxAcc;
	float plateauTime;
	float rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;

	float rampUpDist = rampUpTime * (startSpeed + (maxSpeed - startSpeed)/2.0);
	float plateauDist;
	float rampDownDist = rampDownTime * (maxSpeed + (maxSpeed - finalSpeed)/2.0);


	if (rampUpDist + rampDownDist > pathLength) {
		//	triangle case: we don't ever hit full speed
		
		//	calculate what max speed we actually reach (it's less than the parameter passed in)
		//	we write an equation for pathLength given maxSpeed, then solve for maxSpeed
		//		rampUpTime = (maxSpeed - startSpeed) / maxAcc;
		//		rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;
		//		pathLength = (startSpeed + (maxSpeed - startSpeed)/2)*rampUpTime
		//						+ (maxSpeed + (finalSpeed - maxSpeed)/2)*rampDownTime;
		//		//	we then solve for maxSpeed
		maxSpeed = sqrt(pathLength*maxAcc + startSpeed*startSpeed + finalSpeed*finalSpeed);

		rampUpTime = (maxSpeed - startSpeed) / maxAcc;
		rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;
		rampUpDist = (startSpeed + (maxSpeed - startSpeed)/2) * rampUpTime;
		rampDownDist = (finalSpeed + (maxSpeed - finalSpeed)/2) * rampDownTime;

		//	no plateau
		plateauTime = 0;
		plateauDist = 0;
	} else {
		//	trapezoid case: there's a time where we go at maxSpeed for a bit
		plateauDist = pathLength - (rampUpDist + rampDownDist);
		plateauTime = plateauDist / maxSpeed;
	}


	if (timeIntoLap < 0) {
		///	not even started on the path yet
		posOut = 0;
		speedOut = startSpeed;
		return false;
	} else if (timeIntoLap < rampUpTime) {
		///	on the ramp-up, we're accelerating at @maxAcc
		posOut = 0.5*maxAcc*timeIntoLap*timeIntoLap + startSpeed*timeIntoLap;
		speedOut = startSpeed + maxAcc*timeIntoLap;
		return true;
	} else if (timeIntoLap < rampUpTime + plateauTime) {
		///	we're on the plateau
		posOut = rampUpDist + (timeIntoLap - rampUpTime)*maxSpeed;
		speedOut = maxSpeed;
		return true;
	} else if (timeIntoLap < rampUpTime + plateauTime + rampDownTime) {
		///	we're on the ramp down
		float timeIntoRampDown = timeIntoLap - (rampUpTime + plateauTime);
		posOut = 0.5*(-maxAcc)*timeIntoRampDown*timeIntoRampDown + maxSpeed*timeIntoRampDown + (rampUpDist + plateauDist);
		speedOut = maxSpeed - maxAcc*timeIntoRampDown;
		return true;
	} else {
		///	past the end of the path
		posOut = pathLength;
		speedOut = finalSpeed;
		return false;
	}
}
