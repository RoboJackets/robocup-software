#pragma once

#include "gameplay/Behavior.hpp"

namespace Gameplay
{

class DumbReceive : public ReceivingBehavior
{
public:
	static void createConfiguration(Configuration *cfg);

	DumbReceive(GameplayModule *gameplay);

	bool run();
	bool done();
	void restart();
	bool success();
	bool ready();

private:
	enum State {
		Setup,
		Receive_PassKicking,
		Receive_PassDone,
		Done
	};

	// Return line up position
	Geometry2d::Point receivePosition();

	bool isFailedReceive();

	State _state;
	bool _success;

	static ConfigDouble *_backoffDistance;
	static ConfigDouble *_maxExecutionError;
	static ConfigDouble *_kickTimeout;

	static ConfigBool *_debug;
};

}
