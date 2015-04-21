#include "MightyPass.hpp"

using namespace std;
using namespace Geometry2d;


REGISTER_PLAY_CATEGORY(Gameplay::Plays::MightyPass, "Playing")

namespace Gameplay {
	namespace Plays {
		REGISTER_CONFIGURABLE(MightyPass)
	}
}





ConfigDouble *Gameplay::Plays::MightyPass::_planningHysterisis;


void Gameplay::Plays::MightyPass::createConfiguration(Configuration *cfg)
{
	_planningHysterisis = new ConfigDouble(cfg, "MightyPass/Planning Hysterisis", .2);
}



Gameplay::Plays::MightyPass::MightyPass(GameplayModule *gameplay):
	Play(gameplay),
	_leftDefender(gameplay, Behaviors::Defender::Left),
	_rightDefender(gameplay, Behaviors::Defender::Right),
	_fieldEval(gameplay->state())
{
	_leftDefender.otherDefenders.insert(&_rightDefender);
	_rightDefender.otherDefenders.insert(&_leftDefender);

	_fieldEval.visualize = true; 
}

float Gameplay::Plays::MightyPass::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::MightyPass::run()
{
	Rect rect(Point(-Field_Width / 2.0f, 0), Point(Field_Width / 2.0f, Field_Length));
	_fieldEval.visualize = true;
	_fieldEval.bestPointInRect(rect);


	return true;


	// // handle assignments
	// set<OurRobot *> available = _gameplay->playRobots();

	// // defense first - closest to goal
	// assignNearest(_leftDefender.robot, available, Geometry2d::Point());
	// assignNearest(_rightDefender.robot, available, Geometry2d::Point());

	// // choose offense, we want both robots to attack
	// assignNearest(_kicker1.robot, available, ball().pos);
	// assignNearest(_kicker2.robot, available, ball().pos);

	// // additional defense - if it exists
	// assignNearest(_centerDefender.robot, available, Geometry2d::Point());

	// // manually reset any kickers so they keep kicking
	// // if (_kicker1.done())
	// // 	_kicker1.restart();
	// // if (_kicker2.done())
	// // 	_kicker2.restart();

	// // add obstacles - always bias to kicker1
	// if (_kicker1.robot && _kicker2.robot) {
	// 	unsigned k1 = _kicker1.robot->shell(), k2 = _kicker2.robot->shell();
	// 	_kicker1.robot->avoidTeammateRadius(k2, 0.8 * Robot_Radius);
	// 	_kicker2.robot->avoidTeammateRadius(k1, 0.5);
	// }

	// // execute kickers dumbly
	// if (_kicker1.robot) _kicker1.run();
	// if (_kicker2.robot) _kicker2.run();

	// // run standard defender behavior
	// if (_leftDefender.robot) _leftDefender.run();
	// if (_rightDefender.robot) _rightDefender.run();
	// if (_centerDefender.robot) _centerDefender.run();
	
	// return true;
}



// #pragma mark -

// bool Gameplay::Plays::MightyPass::run() {


// 	bool haveBall = true;	//	FIXME: ?


// 	if ( haveBall ) {
// 		//******	maintain some defensively-positioned bots just in case
// 		//	have a defensiveness score?
// 		float defensiveness = 0;		//	= field length - average OurRobot->pos.
// 		float theirOffensiveness = 0;	//	



// 		//***** build a graph of possible actions that lead from active bot to goal
// 		OurRobot *activeRobot = NULL;	//	FIXME: ?




// 	} else {



// 	}




// }


// #pragma mark Evaluation
// //================================================================================


// float Gameplay::Plays::MightyPass::evaluatePass(Point &from, Point &to) {

// }


// float Gameplay::Plays::MightyPass::evaluateShot(Point &from, Segment &goalSegment) {

// }


// float Gameplay::Plays::MightyPass::evaluateReposition(Point &from, Point &to) {
// 	const float multiplier = 1;	//	FIXME: get a real multiplier
// 	float dist = (to - from).mag();

// 	return dist * multiplier;
// }


