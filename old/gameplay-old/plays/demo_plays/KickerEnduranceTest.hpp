#pragma once

#include <boost/timer.hpp>
#include "../../Play.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * Fires the kicker repeatedly during play, while holding the robot stationary.
 *
 * Designed to diagnose manufacturing flaws in the kicker design.
 */
class KickerEnduranceTest: public Play
{
public:
	static void createConfiguration(Configuration *cfg);

	KickerEnduranceTest(GameplayModule *gameplay);

	virtual bool run();

protected:

	static ConfigBool *_use_chipper; ///< Allows chipper use
	static ConfigInt *_kick_power;   ///< Power for kick commands
	static ConfigDouble *_kick_interval; ///< time to wait in seconds between kicks

	boost::timer _timer; ///< simple timer for kicking
	std::map<unsigned int, size_t> _kicker_counts; ///< draw a kicker count for all robots
};
} // \Gameplay
} // \Plays
