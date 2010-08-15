#include <QColor>
#include "DemoPassPlay.hpp"

//#define TIMEMARGIN 3.5 // seconds a play can deviate from plan before abort
//#define ROBOTSUCCESSMARGIN 0.05 // if robot within this region, a move is complete
//#define ROBOTKICKSUCCESSMARGIN 0.05 // if kicking robot within this region, proceed to kick
//#define BALLSUCCESSMARGIN 0.2 // if robot within this region, a move is complete

using namespace Geometry2d;
using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoPassPlay, "Demos")

Gameplay::Plays::DemoPassPlay::DemoPassPlay(GameplayModule *gameplay)
: Play(gameplay), passPlanner_(gameplay, false)
{
	set<Robot *> available = gameplay->robots();
	passPlanner_.assign(available);
}

bool Gameplay::Plays::DemoPassPlay::run(){
	passPlanner_.enableOptimization(false);
	// set margins
	double time_margin = 5.5,
			robotsuccess_margin = 0.05,
			ballsuccess_margin = 0.2;
	passPlanner_.setMargins(time_margin, robotsuccess_margin, ballsuccess_margin);

	passPlanner_.run();

	return true;
}
