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
		class TestDirectMotionControl: public Play
		{
			public:
				typedef std::vector<Geometry2d::Point> path_t;

				TestDirectMotionControl(GameplayModule *gameplay);

				/** Always applicable if we are playing */
				virtual bool applicable();

				/** Picks exactly one robot */
				virtual bool assign(std::set<Robot *> &available);

				/** Called every frame */
				virtual bool run();

			protected:

				/** Enum for states - need to set up first */
				typedef enum {
					Setup, // move to the start point
					Track  // track the path
				} States;

				/** current state */
				States fsm_state_;

				/** radius of circle */
				double radius_;

				/** start point */
				Geometry2d::Point start_pt_;

				/** Path to execute */
				path_t path_;
		};
	}
}
