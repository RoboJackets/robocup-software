#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/Capture.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class Fling: public SingleRobotBehavior
		{
			public:
				Fling(GameplayModule *gameplay);
				
				static void createConfiguration(Configuration *cfg);

				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				bool capture() const {
					return _state == State_Capture;
				}

				void restart();
				
				// Default target is goal center, and does not account for
				// obstacles in the way of the ball
				// Plays should use window evaluator to choose targets
				Geometry2d::Point target;
				
				// If off, will stay in State_Capture until turned on - use for synchronizing
				// with other robots.  Defaults to true
				bool enable_fling;

				// The speed to drive the dribbler at during aiming
				// and flinging
				short dribble_speed;

				// The angular speed with which to spin
				double spin_speed;

			private:

				enum
				{
					State_Capture,
					State_Pivot,
					State_Fling,
					State_Done
				} _state;
				
				Capture _capture;

				Geometry2d::Point _flingBallStart;   // position of the ball at the start of fling

				bool _ccw;

				// tuning parameters
				static ConfigDouble *_fling_travel_thresh; // minimum distance the ball must travel to be done
				static ConfigDouble *_pivot_Speed;
		};
	}
}
