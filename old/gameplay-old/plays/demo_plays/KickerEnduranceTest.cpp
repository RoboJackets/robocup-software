#include "KickerEnduranceTest.hpp"


using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::KickerEnduranceTest, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(KickerEnduranceTest)
	}
}

ConfigBool *Gameplay::Plays::KickerEnduranceTest::_use_chipper;
ConfigInt  *Gameplay::Plays::KickerEnduranceTest::_kick_power;
ConfigDouble *Gameplay::Plays::KickerEnduranceTest::_kick_interval;

void Gameplay::Plays::KickerEnduranceTest::createConfiguration(Configuration* cfg)
{
	_use_chipper  = new ConfigBool(cfg, "KickerEnduranceTest/Enable Chipping", false);
	_kick_power = new ConfigInt(cfg, "KickerEnduranceTest/Kicker Power", 127);
	_kick_interval = new ConfigDouble(cfg, "KickerEnduranceTest/Kick Interval", 10.0);
}

Gameplay::Plays::KickerEnduranceTest::KickerEnduranceTest(GameplayModule *gameplay):
	Play(gameplay)
{
	_timer.restart();
}

bool Gameplay::Plays::KickerEnduranceTest::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	if (_timer.elapsed() > *_kick_interval) {

		// fire all robots
		for (OurRobot* r :  available) {
			if (*_use_chipper)
				r->chip(*_kick_power);
			else
				r->kick(*_kick_power);

			// track number of kicks
			if (_kicker_counts.find(r->shell()) == _kicker_counts.end()) {
				_kicker_counts[r->shell()] = 1;
			} else {
				_kicker_counts[r->shell()] = _kicker_counts[r->shell()] + 1;
			}

		}
		_timer.restart();
	}

	// draw on screen
	for (OurRobot* r :  available) {
		size_t kicks = 0;
		if (_kicker_counts.find(r->shell()) == _kicker_counts.end()) {
			_kicker_counts[r->shell()] = 0;
		} else {
			kicks = _kicker_counts[r->shell()];
		}
		r->addText(QString("Kicks:  %1").arg(
			QString::number(kicks)));
	}

	return true;
}
