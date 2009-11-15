/*
 * TestBasicPassing.hpp
 *
 *  Created on: Nov 8, 2009
 *      Author: alexgc
 */

#pragma once

#include "../../Play.hpp"

#include <gameplay/behaviors/test/Passer.hpp>
#include <gameplay/behaviors/test/Receiver.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TestBasicPassing: public Play
		{
			public:
				TestBasicPassing(GameplayModule *gameplay);

				// test plays always return applicable
				virtual bool applicable() {return true;}

				/** Passing test needs two robots to work */
				virtual void assign(std::set<Robot *> &available);

				/** default run */
				virtual bool run();

			protected:
				/** one robot is the passer, the other is the receiver
				 * These will switch as necessary
				 */
				Behaviors::Passer _passer;
				Behaviors::Receiver _receiver;

				// trajectory components
				Geometry2d::Line _trajectory;

				// passing system state
				enum State
				{
					CreateTrajectory,
					Positioning,
					Execute,
					Done
				};

				State _passState;
		};
	}
}
