#pragma once

#include "../Behavior.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/Fling.hpp>
#include <gameplay/behaviors/Yank.hpp>
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
		 *  - Fling: tries to fling at a corner of the field
		 *  - Chip: same as kick, but uses the chipper
		 *  - BumpYank: like a yank, but with a bump first - target is one side of field
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
					Mode_Kick,
					Mode_Chip,
					Mode_FlingLeft,
					Mode_FlingRight,
					Mode_BumpYankLeft,
					Mode_BumpYankRight
				} KickoffMode;
				KickoffMode mode;

				bool modeChosen() const { return _mode_chosen; }

				void kickTarget(const Geometry2d::Segment& target) {
					_kick.setTarget(target);
				}

			private:
				Behaviors::Bump _bump;
				Behaviors::Kick _kick;
				Behaviors::Fling _fling;
				Behaviors::Yank _yank;

				bool _mode_chosen; // true when mode has been chosen

				void chooseMode(); // sets the mode at startup

				void executeMode(); // runs the correct behavior

				// GUI Parameters
				static ConfigBool * _enableRandomKick; // if true, chooses among allowable types
				static ConfigBool * _enableFling; // if true, allows for fling to be used - uses side
				static ConfigBool * _enableChip;  // if true, will try to chip at endline
				static ConfigBool * _enableBumpYank;  // if true, will bump forward and yank back - uses side
				static ConfigDouble *_chipMinRange;
				static ConfigDouble *_chipMaxRange;
		};
	}
}
