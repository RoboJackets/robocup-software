#pragma once
#include "planning/Path.hpp"

namespace Planning
{
	/**
	 * @brief Interface for Path Planners
	 */
	class PathPlanner
	{

	public:
		//std::unique_ptr<Path> plan(const Geometry2d::Point& startPos, const Geometry2d::Point& startVel, const Geometry2d::Point& endPos, const Geometry2d::Point& endVel, const MotionConstraints& motionConstraints)=0;
		/**
		 * Virtual function which returns a planned Path following the specified MotionContraints and doesn't hit the obstacles.
		 */
		virtual std::unique_ptr<Path> run(
				MotionInstant startInstant,
				MotionInstant endInstant,
				const MotionConstraints &motionConstraints,
				const Geometry2d::CompositeShape *obstacles)=0;

	};
}