/*
 * TestIntercept.hpp
 *
 * Testing the ball intercept behavior by inserting a pause after interception, then kicking.
 *
 *  Created on: Feb 11 2010
 *      Author: Philip Rogers
 *      Author:
 */
#pragma once

#include "../../Play.hpp"

#include "../../behaviors/Kick.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class TestIntercept: public Play
		{
			public:
				TestIntercept(GameplayModule *gameplay);

				/** returns true if the play is currently applicable given gamestate */
				virtual bool applicable();

				/** Assigns robots to the play given a set of robots */
				virtual void assign(std::set<Robot *> &available);

				/** Called every frame */
				virtual bool run();

			protected:
				// passing system state
				enum State{Initializing,Optimizing,Executing,Done};
				State _state;

				Behaviors::Kick _kicker;
		};
	}
}
