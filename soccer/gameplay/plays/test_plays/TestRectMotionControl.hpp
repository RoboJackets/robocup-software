/**
 *  Play to test motion control using direct path control commands
 */

#pragma once

#include <vector>
#include "../../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class TestRectMotionControl: public Play
		{
			public:
				typedef std::vector<Geometry2d::Point> path_t;

				TestRectMotionControl(GameplayModule *gameplay);

				/** Always applicable if we are playing */
				virtual bool applicable();

				/** Picks exactly one robot */
				virtual bool assign(std::set<Robot *> &available);

				/** Called every frame */
				virtual bool run();

			protected:

				/** Enum for states - need to set up first */
				typedef enum {
					Wait, // waiting at a point on the path
					Track  // tracking along the path
				} States;

				/** current state */
				States fsm_state_;

				/** size of rectangle */
				float hWidth_;
				float hHeight_;

				/** time to wait at each waypoint */
				int waitFramesMax_;

				/** time waiting at current waypoint */
				int waitFrames_;

				/** index of current goal */
				int pathGoalIdx_;

				/** Path to execute */
				path_t path_;
		};
	}
}
