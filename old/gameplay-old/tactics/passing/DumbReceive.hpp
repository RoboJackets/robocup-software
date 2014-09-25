#pragma once

#include <gameplay/ActionBehavior.hpp>
#include <Timeout.hpp>

namespace Gameplay
{

class DumbReceive : public ActionBehavior
{
public:
	static void createConfiguration(Configuration *cfg);

	DumbReceive(GameplayModule *gameplay);

	bool run();

	bool isSettingUp();
	bool isSetup();
	bool isActing();
	bool isDone();
	void restart();




	Timeout timeout;

private:
	enum State {
		Setup,
		Receive_PassKicking,
		Receive_PassDone,
		Done
	};


	// Return line up position
	Geometry2d::Point receivePosition();

	float receivePositionError();

	bool isAtReceivePosition();

	bool isFailedReceive();

	State _state;
	bool _success;
	uint64_t _kickDelta;

	static ConfigDouble *_backoffDistance;
	static ConfigDouble *_maxExecutionError;
	static ConfigDouble *_kickTimeout;

	static ConfigBool *_debug;
};

}
