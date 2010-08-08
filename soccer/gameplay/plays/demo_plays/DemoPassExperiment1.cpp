#include <QColor>
#include "DemoPassExperiment1.hpp"
#include <iostream> // testing
using namespace Geometry2d;
using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoPassExperiment1, "Demos")

Gameplay::Plays::DemoPassExperiment1::DemoPassExperiment1(GameplayModule *gameplay)
: Play(gameplay), passPlanner_(gameplay, true), _kicker1(gameplay), _kicker2(gameplay),
  _expTime(5000000 /* 5 seconds */), _expState(Initializing), _expType(OptimizedPass)
{
}

bool Gameplay::Plays::DemoPassExperiment1::run(){
	long time = _gameplay->state()->timestamp;

	if(_expState==Initializing){
		cout << "Experiment1: Initializing Experiment 1" << endl;
		if(_expType==TwoOffense){
			cout << "Experiment1: Type: TwoOffense" << endl;
			BOOST_FOREACH(Robot *r, _robots){
				if(r->id() == 0){_kicker1.assignOne(r); continue;}
				if(r->id() == 1){_kicker2.assignOne(r); continue;}
			}
			_kicker1.teammate = &_kicker2;
			_kicker2.teammate = &_kicker1;
		}else if(_expType==Pass){
			cout << "Experiment1: Type: Pass" << endl;
			passPlanner_.enableOptimization(false);
			passPlanner_.assign(_robots);
			// set margins
			double time_margin = 5.5,
					robotsuccess_margin = 0.05,
					ballsuccess_margin = 0.2;
			passPlanner_.setMargins(time_margin, robotsuccess_margin, ballsuccess_margin);
		}else if(_expType==OptimizedPass){
			cout << "Experiment1: Type: OptimizedPass" << endl;
			passPlanner_.enableOptimization(true);
			passPlanner_.assign(_robots);
			// set margins
			double time_margin = 5.5,
					robotsuccess_margin = 0.05,
					ballsuccess_margin = 0.2;
			passPlanner_.setMargins(time_margin, robotsuccess_margin, ballsuccess_margin);
		}

		_expState = WaitOppStart;
		cout << "Experiment1: Waiting on opponent to move." << endl;
	}

	if(_expState==WaitOppStart){ // check if opponent moves
		for(int o=0; o<Constants::Robots_Per_Team; o++){
			if(opp(o)->vel().mag() > 0.001){
				_expState = Running;
				_startTime = time;
				cout << "Experiment1: Running." << endl;
			}
		}
	}

	if(_expState==Running){
		if(_expType==TwoOffense){
			_kicker1.run();
			_kicker2.run();
		}else if(_expType==Pass || _expType==OptimizedPass){
			passPlanner_.run();
		}

		if(time - _startTime > _expTime){
			_expState = Done;
			cout << "Experiment1: Done." << endl;
		}
	}

	return true;
}
