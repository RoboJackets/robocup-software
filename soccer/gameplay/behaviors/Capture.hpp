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
			};
	}
}
