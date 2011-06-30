#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/Capture.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class PivotKick: public SingleRobotBehavior
		{
			public:
				PivotKick(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				bool aiming() const {
					return _state == State_Aim;
				}

				bool capture() const {
					return _state == State_Capture;
				}

				void restart();
				
				// Default target is goal segment, but does not take into account
				// obstacles.  Plays should use window evaluator to kick
				Geometry2d::Segment target;
				
				// If off, will stay in State_Aim until turned on - use for synchronizing
				// with other robots.  Defaults to true
				bool enable_kick;

				// If on, the robot will get desperate after aiming for too long and
				// kick the ball downfield. Defaults to true
				bool enable_desparate_kick;

				// The speed to drive the dribbler at during aiming
				// If high, adds lift to kick
				// Default: full speed
				short dribble_speed;

				// If false, uses straight kicker, if true, uses chipper
				// Default: false
				bool use_chipper;

				// Allows for different kicker settings, such as for
				// passing with lower power.
				// Default: 255 - full power
				uint8_t kick_power;

			private:

				enum
				{
					State_Capture,
					State_Aim,
					State_Done
				} _state;
				
				Capture _capture;

				bool _ccw;
				float _lastError;
				float _lastDelta;
				uint64_t _lastBallTime;
				float _accuracy;
				bool _kicked;
			};
	}
}
