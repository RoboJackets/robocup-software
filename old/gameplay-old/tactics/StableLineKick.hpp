#pragma once

#include <gameplay/ActionBehavior.hpp>

namespace Gameplay
{

class StableLineKick : public ActionBehavior
{
public:
	static void createConfiguration(Configuration *cfg);

	StableLineKick(GameplayModule *gameplay);

	bool run();

	bool isSettingUp();
	bool isSetup();
	bool isActing();
	bool isDone();
	void restart();

private:
	enum State {
		Setup,
		Kick,
		Done
	};

	// Super-class variable
	// actionTarget == Point to receive pass

	// Point to start passing from
	Geometry2d::Point passPosition();

	float passPositionError();
	float passPathError();

	bool isAtPassPosition();  //
	bool isOnPassPath();

	State _state;

	static ConfigInt *_kickPower;
	static ConfigDouble *_backoffDistance;
	static ConfigDouble *_maxExecutionError;
	static ConfigDouble *_approachVelocity;

	static ConfigBool *_debug;
};

}
