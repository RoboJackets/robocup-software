#pragma once

#include <gameplay/ActionBehavior.hpp>

namespace Gameplay
{

class StablePass : public ActionBehavior
{
public:
	static void createConfiguration(Configuration *cfg);

	StablePass(GameplayModule *gameplay);

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

	// Pass-positioning functions
	Geometry2d::Point compute_PassTarget();
	Geometry2d::Point compute_PassPosition();
	Geometry2d::Point compute_PassDirection();
	double error_PassPosition();
	double error_PassDirection();
	double error_PassPath();
	bool ok_PassPosition();
	bool ok_PassDirection();
	bool ok_Pass(); // Both position and direction are within acceptable deviations
	bool ok_PassPath(); // Checks error on the path deviation
	bool ok_PassApproach(); // Both path and direction are within acceptable deviations


	State _state;

	static ConfigDouble *_ERROR_PASSPOSITION;
	static ConfigDouble *_ERROR_PASSDIRECTION;
	static ConfigDouble *_ERROR_PASSPATH;

	static ConfigInt *_KICKPOWER;
	static ConfigDouble *_BACKOFF_DISTANCE;
	static ConfigDouble *_APPROACH_VELOCITY;

	static ConfigBool *_DEBUG;
};

}
