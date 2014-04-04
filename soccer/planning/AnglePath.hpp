#pragma once

#include "TrapezoidalMotion.hpp"

namespace Planning {
	/**
	 * Provides instruction for how to rotate the robot as a function of position given time.
	 */
	class AnglePath {
	public:
		/**
		 * Create a new angle path based on trapezoidal motion
		 */
		AnglePath(float startAngle, float startAngleVel, float endAngle, endAngleVel = 0);
		
		/**
		 * Find the angle and velocity the robot should be at given time @t
		 * @param t Time (in seconds) since starting the path
		 */
		bool evaluate(float t, float &angleOut, float &angleVelOut);
	};
}
