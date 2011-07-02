#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/Capture.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class Yank: public SingleRobotBehavior
		{
			public:
				Yank(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				bool capture() const {
					return _state == State_Capture;
				}

				void restart();
				
				// Default target is goal segment, but does not take into account
				// obstacles.  Plays should use window evaluator to choose targets
				Geometry2d::Segment target;
				
				// If off, will stay in State_Capture until turned on - use for synchronizing
				// with other robots.  Defaults to true
				bool enable_yank;

				// The speed to drive the dribbler at during aiming
				// and yanking
				short dribble_speed;

			private:

				enum
				{
					State_Capture,
					State_Yank,
					State_Done
				} _state;
				
				Capture _capture;

				Geometry2d::Point _yankBallStart;   // position of the ball at the start of yank

				// tuning parameters
				ConfigDouble::shared_ptr _yank_travel_thresh; // minimum distance the ball must travel to be done
				ConfigDouble::shared_ptr _max_aim_error;      // maximum distance from yank line allowed
				ConfigDouble::shared_ptr _backup_dist; 				// minimum distance to clear the ball before getting off line
				ConfigDouble::shared_ptr _ball_clearance;     // distance the robot needs to get away from the line
		};
	}
}
