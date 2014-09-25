#pragma once

#include "../Behavior.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/Bump.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		/**
		 * General kickoff behavior with several modes for
		 * ways to approach the ball and determine the kicking
		 * procedure
		 *
		 * Modes:
		 *  - Default: Pivot Kick: tries to kick at end line
		 *  	- To force, disable other modes and randomness
		 *  - Chip: same as kick, but uses the chipper
		 *
		 * Use the parameters to choose kickoff mode
		 */
		class Kickoff: public SingleRobotBehavior
		{
			public:
				Kickoff(GameplayModule *gameplay);
				
				static void createConfiguration(Configuration *cfg);

				virtual bool run();

				// MODE setting - this stores which play will be used
				typedef enum {
					Mode_None,
					Mode_Bump,
					Mode_Kick,
					Mode_KickLeftCorner,
					Mode_KickRightCorner,
					Mode_Chip,
					Mode_ChipLeftCorner,
					Mode_ChipRightCorner,
				} KickoffMode;
				KickoffMode mode;

				void kickTarget(const Geometry2d::Segment& target) {
					_kick.setTarget(target);
				}

				// flags controlled by top-level play
				bool useRandomKick;
				uint8_t kickPower;
				bool enableChip;
				bool enableBump;

			private:
				Behaviors::Kick _kick;
				Behaviors::Bump _bump;

				void chooseMode(); // sets the mode at startup

				void executeMode(); // runs the correct behavior

				// GUI Parameters
				static ConfigBool * _enableChip;  // if true, will try to chip at endline
				static ConfigDouble *_chipMinRange;
				static ConfigDouble *_chipMaxRange;
		};
	}
}
