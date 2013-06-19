#include "PassReceiveTactic.hpp"

Gameplay::PassReceiveTactic::PassReceiveTactic(GameplayModule *gameplay)
	: TwoRobotBehavior(gameplay),
	  leader(0),
	  receiver(0),
	  passAction(0),
	  receiveAction(0),
	  receiveTarget(0,0),
	  _state(Setup)
{
}

Gameplay::PassReceiveTactic::~PassReceiveTactic()
{
}

bool Gameplay::PassReceiveTactic::isDone()
{
	return passAction->isDone() && receiveAction->isDone();
}

void Gameplay::PassReceiveTactic::setActions(ActionBehavior* pass, ActionBehavior* receive)
{
	passAction = pass;
	receiveAction = receive;
	setFields();
}

void Gameplay::PassReceiveTactic::setRobots(OurRobot *l, OurRobot *r)
{
	leader = l;
	receiver = r;
	setFields();
}

void Gameplay::PassReceiveTactic::pointToReceive(const Geometry2d::Point& target)
{
	receiveTarget = target;
	setFields();
}

void Gameplay::PassReceiveTactic::restart()
{
	passAction->restart();
	receiveAction->restart();
}

void Gameplay::PassReceiveTactic::setFields()
{
	passAction->partner = receiveAction;
	receiveAction->partner = passAction;

	passAction->actionTarget = receiveTarget;
	receiveAction->actionTarget = receiveTarget;

	passAction->robot = leader;
	receiveAction->robot = receiver;
}

bool Gameplay::PassReceiveTactic::positionsFilled()
{
	if(!passAction->robot || !receiveAction->robot) {
		return false;
	}

	//if(!passAction->robot->visible || receiveAction->robot->visible) {
	//	return false;
	//}

	return true;
}

bool Gameplay::PassReceiveTactic::run()
{
	if(!positionsFilled()) {
		return false;
	}

	passAction->run();
	receiveAction->run();

	return true;
}
