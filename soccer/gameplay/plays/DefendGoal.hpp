/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/positions/Forward.hpp>
#include <gameplay/behaviors/GoalDefender.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class DefendGoal: public Play
		{
			public:
				typedef enum
				{
					MarkAndDefend,
					DefendAndDefend,
					DefendAndMark
				} DefenseState;

				DefendGoal(GameplayModule *gameplay);

				/** returns true if the play is currently applicable given gamestate */
				virtual bool applicable(const std::set<Robot *> &robots);

				/** Assigns robots to the play given a set of robots */
				virtual bool assign(std::set<Robot *> &available);

				/** Called every frame */
				virtual bool run();

			protected:
				DefenseState _defenseState;

				Behaviors::Fullback _fullback1, _fullback2;
				Behaviors::Forward _kicker1, _kicker2;
				Behaviors::GoalDefender _goalDefender;
		};
	}
}
