#pragma once

#include <stdint.h>
#include <vector>

#include <Geometry2d/Point.hpp>

class MotionCmd
{
	public:
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
		
		enum PivotType
		{
			NoPivot = 0,
			CW = 1,
			CCW = 2
		};
		
		enum SpinType
		{
			NoSpin = 0,
			SpinCW = 1,
			SpinCCW = 2
		};
		
		OrientationType face;
		PivotType pivot;
		Geometry2d::Point pivotPoint;
		SpinType spin;
		enum PlannerType
		{
			Point = 0,
			DirectVelocity = 1,
			DirectMotor = 2,
			ForceStop = 3
		};
		
		PlannerType planner;
		enum PathEndType
		{
			StopAtEnd = 0,
			FastAtEnd = 1
		};
		
		PathEndType pathEnd;

		/** the robot velocity - output of PointController */
		float direct_ang_vel;
		Geometry2d::Point direct_trans_vel;

		/** the motor commands - output of WheelControlModule */
		std::vector<int8_t> direct_motor_cmds;
		
		MotionCmd()
		{
			pathLength = 0.0;
			vScale = 1.0;
			wScale = 1.0;
			face = None;
			pivot = NoPivot;
			spin = NoSpin;
			planner = Point;
			pathEnd = StopAtEnd;
			direct_ang_vel = 0.0;
		}
};
