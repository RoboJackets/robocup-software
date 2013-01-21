#pragma once

#include "gameplay/Behavior.hpp"

namespace Gameplay
{

class StableLineKick : public PassingBehavior
{
public:
	static void createConfiguration(Configuration *cfg);

	StableLineKick(GameplayModule *gameplay);

	bool run();
	bool done();
	bool kicking();
	bool setup();
	void restart();

private:
	enum State {
		Setup,
		Kick,
		Done
	};

	// Point to start passing from
	Geometry2d::Point passPosition();

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
