#pragma once

#include <gameplay/ActionBehavior.hpp>

namespace Gameplay
{

class PassReceiveTactic : public TwoRobotBehavior
{
public:
	PassReceiveTactic(GameplayModule *gameplay);
	~PassReceiveTactic();

	bool run();

	void restart();

	bool isDone();

	bool positionsFilled();

	void setActions(ActionBehavior* pass, ActionBehavior* recieve);
	void setRobots(OurRobot *leader, OurRobot *receiver);
	void pointToReceive(const Geometry2d::Point& target);

	ActionBehavior *pass() { return passAction; }
	ActionBehavior *receive() { return receiveAction; }

private:
	enum State {
		Setup,
		Action,
		Done
	};

	void setFields();

	OurRobot *leader;
	OurRobot *receiver;

	ActionBehavior *passAction;	   // no ownership
	ActionBehavior *receiveAction;  // no ownership

	Geometry2d::Point receiveTarget;

	State _state;
};

}
