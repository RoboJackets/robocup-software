#include "OptimizedOffense.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OptimizedOffense, "Playing")

Gameplay::Plays::OptimizedOffense::OptimizedOffense(GameplayModule *gameplay):
	Play(gameplay)
{
}

float Gameplay::Plays::OptimizedOffense::score ( Gameplay::GameplayModule* gameplay )
{
	return 0;
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
