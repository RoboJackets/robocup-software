/**
 *  Play to test motion control using time-position control
 *  This uses the new packet datatype with Position models tagged to a timestamp
 */

#pragma once

#include <vector>
#include "../../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class TestTimePositionControl: public Play
		{
			public:
				typedef std::vector<Geometry2d::Point> path_t;

				TestTimePositionControl(GameplayModule *gameplay);

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

				/** Initial time */
				uint64_t start_time_;
		};
	}
}
