#pragma once

#include "../../Play.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/Move.hpp>

#include <QTime>

namespace Gameplay
{
	namespace Plays
	{
		class DemoPassing: public Play
		{
			public:
				DemoPassing(GameplayModule *gameplay);
				
				virtual bool run();

			protected:
				/** one robot is the passer, the other is the receiver
				 * These will switch as necessary
				 */
				Behaviors::Kick _passer;
				Behaviors::Move _receiver;

				QTime _doneTime;
		};
	}
}
