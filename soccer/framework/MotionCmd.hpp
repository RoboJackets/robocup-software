#pragma once

#include <stdint.h>
#include <vector>

#include <Geometry2d/Point.hpp>

//FIXME - Rename to MotionCommand.  If you don't like it, use a better editor.
class MotionCmd
{
	public:
		//FIXME - Remove pathLength and pathEnd.  Store a path in MotionCmd.  What about facing?
		
		/** Inputs to PointController */
		Geometry2d::Point goalPosition;
		Geometry2d::Point goalOrientation;
		float pathLength;

		float vScale;
		float wScale;
		
		enum OrientationType
		{
			None = 0,
			Continuous = 1,
			Endpoint = 2
		};
		OrientationType face;
		
		enum PlannerType
		{
			Point = 0,
			DirectVelocity = 1,
			DirectMotor = 2
		};
		PlannerType planner;
		
		enum PathEndType
		{
			StopAtEnd = 0,
			FastAtEnd = 1
		};
		PathEndType pathEnd;

		float direct_ang_vel;
		Geometry2d::Point direct_trans_vel;
		std::vector<int8_t> direct_motor_cmds;
		
		MotionCmd()
		{
			pathLength = 0.0;
			vScale = 1.0;
			wScale = 1.0;
			face = None;
			planner = Point;
			pathEnd = StopAtEnd;
			direct_ang_vel = 0.0;
		}
};
