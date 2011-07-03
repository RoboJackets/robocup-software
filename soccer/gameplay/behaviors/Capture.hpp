#pragma once

#include <gameplay/Behavior.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		/**
		 * Drives to the ball and gets it under control,
		 * then (coarsely) pivots towards a goal
		 */
		class Capture: public SingleRobotBehavior
		{
			public:
			Capture(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}

				bool approach() const {
					return _state == State_Approach;
				}

				bool capture() const {
					return _state == State_Capture;
				}

				bool pivoting() const {
					return _state == State_Pivoting;
				}

				void restart();
				
				/** goal for facing after trapping ball */
				Geometry2d::Point target;
				
				// if true, will face the target point, otherwise skips to done when pivot is complete
				bool enable_pivot;

			private:
				enum
				{
					State_Approach,
					State_Capture,
					State_Pivoting,
					State_Done
				} _state;
				
				bool _ccw;
				uint64_t _lastBallTime;

				// GUI Config parameters
				ConfigDouble::shared_ptr _stationaryMaxSpeed; // largest ball speed in m/s to treat as stationary

				// How far away from the ball the approach point is placed
				ConfigDouble::shared_ptr _approach_Distance;
				// Ball avoidance distance
				ConfigDouble::shared_ptr _approach_Clearance; // needs to be less than Approach_Threshold
				// How close we must get to the approach point to proceed to Capture
				ConfigDouble::shared_ptr _approach_Threshold;

				// How fast we drive towards the ball
				ConfigDouble::shared_ptr _capture_Speed;
				// How long we must continuously hold the ball to proceed to Aim
				ConfigDouble::shared_ptr _capture_Time_Threshold;
				// How much of Capture_Time_Threshold should be spent decelerating
				ConfigDouble::shared_ptr _capture_Decel;

				// How close the ball must be to count as captured properly
				ConfigDouble::shared_ptr _has_Ball_Dist;

				// Angular speed for Pivoting
				ConfigDouble::shared_ptr _pivot_Speed;

				// Dribbler speed during capture
				ConfigDouble::shared_ptr _dribble_Speed;
			};
	}
}
