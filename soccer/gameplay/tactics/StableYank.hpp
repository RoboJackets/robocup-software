/*
 * StableYank.hpp
 *
 *  Created on: Jan 23, 2013
 *      Author: Matthew Barulic
 */

#ifndef STABLEYANK_HPP_
#define STABLEYANK_HPP_

#include <gameplay/ActionBehavior.hpp>
#include <gameplay/behaviors/Capture.hpp>

namespace Gameplay {

class StableYank: public Gameplay::ActionBehavior {
public:
	static void createConfiguration(Configuration *cfg);

	StableYank(GameplayModule *gameplay);

	bool run();

	bool isSettingUp();
	bool isSetup();
	bool isActing();
	bool isDone();
	void restart();

private:

	enum State {
		Capture,
		Aim,
		Yank,
		Done
	};

	State _state;

	Behaviors::Capture *_capture;

	static ConfigInt *_dribblerPower;
	static ConfigDouble *_yankDistance;
	static ConfigDouble *_aimError;
	static ConfigDouble *_moveError;
	static ConfigBool *_debug;

	Geometry2d::Point _phase1StartPos;

	// Provides the point for capture to aim at
	Geometry2d::Point getCaptureTarget();

	// Defines the point at which phase 1 ends
	Geometry2d::Point getPhase1Target();

	bool isFacingCaptureTarget();
	float faceDistFromCaptureTarget();

};

} /* namespace Gameplay */
#endif /* STABLEYANK_HPP_ */
