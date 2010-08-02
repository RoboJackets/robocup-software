/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#include "ExamplePlay.hpp"

using namespace std;

REGISTER_PLAY(Gameplay::Plays::ExamplePlay)
// Alternatively:
//REGISTER_PLAY_CATEGORY(Gameplay::Plays::ExamplePlay, "Category")

Gameplay::Plays::ExamplePlay::ExamplePlay(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::ExamplePlay::applicable(const std::set<Robot *> &robots)
{
	/** Replace this with code to determine whether this play is applicable now */
	return true;
}

bool Gameplay::Plays::ExamplePlay::assign(set<Robot *> &available)
{
	/** replace this with your code to do assignments */
	/** don't forget to check that the assigns return true, and that
	 * you assign _robots all robots that are assigned
	 */
	return true;
}

bool Gameplay::Plays::ExamplePlay::run()
{
	/** Replace this with your code to run every frame */
	return true;
}
