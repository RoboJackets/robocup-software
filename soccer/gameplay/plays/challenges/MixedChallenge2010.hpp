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

				// start side of the field
				typedef enum {
					LEFT,
					RIGHT
				} FieldSide;
				FieldSide _fieldSide;

				// states of system
				typedef enum {
					SETUP 		/// when we move to the staring position on the home side of the field
				} State;
				State _state;

				// true if kicking is active, otherwise, we just take positions on the field
				bool _usingKicker1;
				bool _usingKicker2;

				Behaviors::Kick _kicker1, _kicker2;
		};
	}
}
