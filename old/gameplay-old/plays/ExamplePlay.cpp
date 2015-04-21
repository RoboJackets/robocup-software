
#include "ExamplePlay.hpp"

using namespace std;

REGISTER_PLAY(Gameplay::Plays::ExamplePlay)
// Alternatively:
//REGISTER_PLAY_CATEGORY(Gameplay::Plays::ExamplePlay, "Category")

Gameplay::Plays::ExamplePlay::ExamplePlay(GameplayModule *gameplay):
	Play(gameplay)
{
}

float Gameplay::Plays::ExamplePlay::score(GameplayModule *gameplay)
{
	return 0;
}

bool Gameplay::Plays::ExamplePlay::run()
{
	/** Replace this with your code to run every frame */
	return true;
}
