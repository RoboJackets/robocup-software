#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class MixedChallenge2010: public Play
		{
			public:
				MixedChallenge2010(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
				
			protected:

				// states of system
				typedef enum {
					STOPPED,	/// initial case
					SETUP, 		/// move to quadrants
					WAITONPASS, /// waiting on them to pass
					RECEIVING,  /// prep a robot for getting a pass
					PASSING,	/// sending a pass someplace
					SHOOTING	/// making a shot ourselves
				} State;
				State _state;

				// true if kicking is active, otherwise, we just take positions on the field
				bool _usingKicker1;
				bool _usingKicker2;

				Behaviors::Kick _kicker1, _kicker2;
		};
	}
}
