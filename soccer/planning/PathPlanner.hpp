#pragma once

namespace Planning
{
	/**
	 * @brief Interface for Path Planners
	 */
	class Path
	{

	public:
		//std::unique_ptr<Path> plan(const Geometry2d::Point& startPos, const Geometry2d::Point& startVel, const Geometry2d::Point& endPos, const Geometry2d::Point& endVel, const MotionConstraints& motionConstraints)=0;
		/**
		 * Virtual function which returns a planned Path following the specified MotionContraints and doesn't hit the obstacles.
		 */
		virtual Planning::InterpolatedPath* run(
				const Geometry2d::Point &start,
				const float angle,
				const Geometry2d::Point &vel,
				const MotionConstraints &motionConstraints,
				const Geometry2d::CompositeShape *obstacles)=0;

	};
}