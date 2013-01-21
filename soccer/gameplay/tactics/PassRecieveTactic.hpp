#pragma once

#include <gameplay/Behavior.hpp>

namespace Gameplay
{

class PassRecieveTactic : public TwoRobotBehavior
{
public:
	PassRecieveTactic(GameplayModule *gameplay);

	bool run();

	OurRobot *passer;
	OurRobot *reciever;

	Geometry2d::Point receiveTarget;

	// PassingBehavior *passBehavior;
	// ReceivingBehavior *recieveBehavior;

private:
	enum State {
		Setup,
		Recieve,
		Done
	};

	State _state;
};

}
