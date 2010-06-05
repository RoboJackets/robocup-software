/**
 * @file Pose.hpp
 * @brief Provides functions for modeling a robot position (Point + angle)
 * @author Alex Cunningham
 */

#pragma once

namespace Geometry2d
{
	/**
	 * Position with orientation
	 *   Uses:
	 *    - robot pose (not velocity)
	 *    - checks for ball given a direction (useful for moving balls)
	 * Utility functions allow for detecting quadrants where a ball might be
	 *
	 * Note: this is not a velocity, as the angle is fixed to be on [-pi, pi]
	 */
	class Pose {
	protected:
		Point _pos;
		float _angle;
		TransformMatrix _transform;

	public:
		/** Constructors */
		Pose(float x, float y, float theta)
			: _pos(x, y), _angle(Utils::fixAngleRadians(theta)),
			  _transform(_pos, theta) {}
		Pose(const Point& pos, float theta)
			: _pos(pos), _angle(Utils::fixAngleRadians(theta)),
			  _transform(pos, theta) {}



	};
} // \namespace Geometry2d


