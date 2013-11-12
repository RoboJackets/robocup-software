#include <PassRecieveTactic.hpp>

Gameplay::PassRecieveTactic::PassRecieveTactic(GameplayModule *gameplay)
	: TwoRobotBehavior(gameplay),
	  passer(0),
	  reciever(0),
	  receiveTarget(0,0),
	  _state(Setup)
{
}

bool Gameplay::PassRecieveTactic::run()
{
	if(!passer || !reciever || _state == Done) {
		return false;
	}



	return false;
}
