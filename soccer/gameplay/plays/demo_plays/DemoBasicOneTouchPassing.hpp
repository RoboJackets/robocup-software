#pragma once

#include "../../Play.hpp"

#include <gameplay/behaviors/OneTouchKick.hpp>
#include <gameplay/behaviors/Move.hpp>

#include <QTime>

namespace Gameplay
{
	namespace Plays
	{
		class DemoBasicOneTouchPassing: public Play
		{
			public:
				DemoBasicOneTouchPassing(GameplayModule *gameplay);

				/** Passing test needs two robots to work */
				virtual bool assign(std::set<Robot *> &available);

				/** default run */
				virtual bool run();

			protected:
				/** one robot is the passer, the other is the receiver
				 * These will switch as necessary
				 */
				Behaviors::OneTouchKick _passer;
				Behaviors::Move _receiver;

				QTime _doneTime;
		};
	}
}
