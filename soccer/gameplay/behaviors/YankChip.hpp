#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/Capture.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		/**
		 * This behavior is designed to escape a slow-dancing scenario
		 * by yanking the ball backwards.
		 *
		 * This is primarily a behavior to escape stuck balls, so
		 *
		 */
		class YankChip: public SingleRobotBehavior
		{
			public:
			static void createConfiguration(Configuration *cfg);

				YankChip(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				bool capture() const {
					return _state == State_Capture;
				}

				void restart();
				
				// Point to try to face towards as a coarse aiming
				Geometry2d::Point target;

				// if false, just backs up and chips in the current direction
				// use if no room for aiming
				bool enable_aiming;

				// If off, will stay in State_Capture until turned on - use for synchronizing
				// with other robots.  Defaults to true
				bool enable_yank;

				// The speed to drive the dribbler at during aiming
				// and yanking
				short dribble_speed;

				// The chip strength to chip over the robot
				uint8_t chip_speed;

				// the distance to back up before chipping
				double backup_distance;

				// opponent robot to chip over - if not available, will use initial ball position
				OpponentRobot * oppRobot;

			private:

				enum
				{
					State_Capture,
					State_Yank,
					State_Chip,
					State_Done
				} _state;
				
				Capture _capture;

				bool _kicked;

				Geometry2d::Point _yankBallStart;   // position of the ball at the start of yank
				Geometry2d::Point _yankRobotStart;  // position of the robot at start of yank

				// tuning parameters
				static ConfigDouble *_yank_travel_thresh; // minimum distance the ball must travel to be done
				static ConfigDouble *_max_aim_error;      // maximum distance from yank line allowed

				static ConfigDouble *_chip_Complete_Dist;
				static ConfigDouble *_min_Ball_Velocity;
				static ConfigDouble *_max_Yank_Dist;
		};
	}
}
