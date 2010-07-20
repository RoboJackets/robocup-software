#pragma once

#include <stdint.h>
#include <vector>

#include <Geometry2d/Point.hpp>

class MotionCmd
{
	public:
		Geometry2d::Point goalPosition;
		Geometry2d::Point goalOrientation;
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
			RRT = 0,
			Path = 1,
			DirectVelocity = 2,
			TimePosition = 3,
			Bezier = 4,
			DirectMotor = 5
		};
		
		PlannerType planner;
		enum PathEndType
		{
			StopAtEnd = 0,
			FastAtEnd = 1
		};
		
		PathEndType pathEnd;
		std::vector<Geometry2d::Point> explicitPath;
		float direct_ang_vel;
		Geometry2d::Point direct_trans_vel;
		std::vector<int8_t> direct_motor_cmds;
		class PathNode
		{
			public:
				float time;
				Geometry2d::Point pos;
				float rot;
				
				PathNode()
				{
					time = 0;
					rot = 0;
				}
		};
		
		std::vector<PathNode> timePosPath;
		uint64_t start_time;
		bool enableBezierAvoid;
		std::vector<Geometry2d::Point> bezierControlPoints;
		
		MotionCmd()
		{
			vScale = 1.0;
			wScale = 1.0;
			face = None;
			pivot = NoPivot;
			spin = NoSpin;
			planner = RRT;
			pathEnd = StopAtEnd;
			direct_ang_vel = 0.0;
			start_time = 0;
			enableBezierAvoid = 0;
		}
};
