#pragma once

#include <gameplay/Behavior.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class TouchKick: public SingleRobotBehavior
		{
			public:
				static void createConfiguration(Configuration *cfg);

				TouchKick(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				void restart();
				
				Geometry2d::Point target;
				
				/** kick parameter flags */
				bool use_chipper;
				uint8_t kick_power;
				bool kick_ready;
				bool enable_kick;


				// scale the kicking parameters to adjust speed/precision of the kick
				float targetRot;

			private:
				enum
				{
					State_Setup,
					State_Ready,
					State_Done
				} _state;

				bool ballClose;


				static ConfigDouble *_done_thresh;
				static ConfigDouble *_rotate_thresh;
				static ConfigDouble *_proj_time;
				static ConfigDouble *_dampening;
		};
	}
}
