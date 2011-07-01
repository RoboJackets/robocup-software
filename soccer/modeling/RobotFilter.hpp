#pragma once

#include <Robot.hpp>

class RobotObservation
{
public:
	RobotObservation(Geometry2d::Point pos = Geometry2d::Point(), float angle = 0, uint64_t time = 0):
		pos(pos), angle(angle), time(time)
	{
	}

	Geometry2d::Point pos;
	float angle;
	uint64_t time;
	
	// Compares the times on two observations.  Used for sorting.
	bool operator<(const RobotObservation &other) const
	{
		return time < other.time;
	}
};

class RobotFilter
{
public:
	RobotFilter();
	
	// Gives a new observation to the filter
	void update(const RobotObservation *obs);
	
	// Generates a prediction of the ball's state at a given time in the future.
	// This may clear robot->visible if the prediction is too long in the future to be reliable.
	void predict(uint64_t time, Robot *robot);

private:
	RobotPose _estimate;
};
