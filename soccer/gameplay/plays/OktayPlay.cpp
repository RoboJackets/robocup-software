/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#include "OktayPlay.hpp"

using namespace std;

Gameplay::Plays::OktayPlay::OktayPlay(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::OktayPlay::applicable()
{
	/** Replace this with code to determine whether this play is applicable now */
	return true;
}

void Gameplay::Plays::OktayPlay::assign(set<Robot *> &available)
{
	/** replace this with your code to do assignments */
}

bool Gameplay::Plays::OktayPlay::run()
{
	/** Replace this with your code to run every frame */
	return true;
}
