#pragma once

#include <gameplay/Behavior.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		/**
		 * Pushes the ball by bumping it
		 */
		class Bump: public SingleRobotBehavior
		{
			public:
				static void createConfiguration(Configuration *cfg);

				Bump(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				void restart();
				
				Geometry2d::Point target;
				
			private:
				enum
				{
					State_Setup,
					State_Charge,
					State_Done
				} _state;

				static ConfigBool *_face_ball;
				static ConfigDouble *_drive_around_dist;
				static ConfigDouble *_setup_to_charge_thresh;
				static ConfigDouble *_escape_charge_thresh;
				static ConfigDouble *_setup_ball_avoid;
				static ConfigDouble *_bump_complete_dist;
				static ConfigDouble *_facing_thresh;
				static ConfigDouble *_accel_bias;
		};
	}
}
