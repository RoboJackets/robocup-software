#pragma once

#include "../Play.hpp"

#include "../behaviors/Kick.hpp"
#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/positions/Forward.hpp>

namespace Gameplay
{
namespace Plays
{
class ClearBall: public Play
{
public:
	ClearBall(GameplayModule *gameplay);

	/**
	 * Applicable any time that the game is running
	 */
	virtual bool applicable(const std::set<Robot *> &robots);

	/**
	 * Takes the first robot as a kicker, ignores the rest
	 */
	virtual bool assign(std::set<Robot *> &available);

	/** default run */
	virtual bool run();

	virtual float score();

protected:

	virtual float score(Robot *r);

	Behaviors::Kick _kicker;
	Behaviors::Fullback _fullback1;
	Behaviors::Forward _kicker1, _kicker2;
	float _oppDistMin; // minimum distance opp must be for score calc
	float _selfDistMax; // maximum distance self robot must be for score calc
	bool _done;
};
} // \Gameplay
} // \Plays
