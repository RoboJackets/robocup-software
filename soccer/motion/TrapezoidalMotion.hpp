#pragma once

/**
 * This function evaluates a motion situation using "bang-bang".
 * The robot accelerates at maxAcceleration until it hits max
 * speed, then decelerates at max acc to the end point.
 *
 * @param pathLength The total distance we are trying to travel
 * @param timeIntoLap How long since we started moving along the trapezoid
 * @param finalSpeed The speed we'd like to be going at the end of the trapezoid
 * @param distOut The position to be at at the given time
 * @param speedOut The speed to be at at the given time
 * @return true if the trapezoid is valid at the given time, false otherwise
 */
bool TrapezoidalMotion(
	float pathLength,
	float maxSpeed,
	float maxAcc,
	float timeIntoLap,
	float startSpeed,
	float finalSpeed,
	float &posOut,
	float &speedOut);

namespace Trapezoidal
{
	float getTime(
		float pathLength,
		float maxSpeed,
		float maxAcc,
		float pos,
		float startSpeed,
		float finalSpeed);
}