#pragma once

#include <gameplay/tactics/ActionBehavior.hpp>

namespace Gameplay
{

class PassRecieveTactic : public TwoRobotBehavior
{
public:
	PassRecieveTactic(GameplayModule *gameplay);

	bool run();

	bool positionsFilled();

	ActionBehavior *passAction;
	ActionBehavior *receiveAction;

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
