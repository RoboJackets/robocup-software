#include "TestIntercept.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestIntercept, "Tests")

Gameplay::Plays::TestIntercept::TestIntercept(GameplayModule *gameplay):
	Play(gameplay, 1), _kicker(gameplay)
{
}

bool Gameplay::Plays::TestIntercept::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

bool Gameplay::Plays::TestIntercept::assign(set<Robot *> &available)
{
	return _kicker.assign(available);
}

bool Gameplay::Plays::TestIntercept::run()
{
	// check if the robot is in done state
	if (_kicker.getState() == Gameplay::Behaviors::Kick::Done)
		_kicker.restart();

	// run the kick play
	_kicker.run();
	return true;
}
