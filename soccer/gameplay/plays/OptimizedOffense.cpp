#include "OptimizedOffense.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OptimizedOffense, "Playing")

Gameplay::Plays::OptimizedOffense::OptimizedOffense(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::OptimizedOffense::applicable(const std::set<Robot *> &robots)
{
	/** Replace this with code to determine whether this play is applicable now */
	return true;
}

bool Gameplay::Plays::OptimizedOffense::run()
{
	if (planReady_) {
		// do small check of viability
		// if acceptable, run the underlying behavior and return
		// otherwise, jump to new plan creation
	} else {
		// create a new plan
	}
	return true;
}
