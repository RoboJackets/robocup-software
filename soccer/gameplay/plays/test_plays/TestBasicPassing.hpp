#pragma once

#include "../../Play.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/Move.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TestBasicPassing: public Play
		{
			public:
				TestBasicPassing(GameplayModule *gameplay);

				virtual bool applicable();

				/** Passing test needs two robots to work */
				virtual void assign(std::set<Robot *> &available);

				/** default run */
				virtual bool run();

			protected:
				/** one robot is the passer, the other is the receiver
				 * These will switch as necessary
				 */
				Behaviors::Kick _passer;
				Behaviors::Move _receiver;

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
