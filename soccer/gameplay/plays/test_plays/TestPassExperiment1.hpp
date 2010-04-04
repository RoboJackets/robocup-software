/*
 * TestPassExperiment1.hpp
 *
 * Runs a non-optimized pass against another team
 *
 * In the best scenario, we would use the referee application, but instead we
 * start this pass experiment which waits for the blue to move before starting.
 * This way, yellow starts the instant blue makes a move, and we can have
 * repeatable experiments without the ability to start both simultaneously.
 *
 * Steps to run experiment:
 *   1.) Start simulator: ./soccsim -c ../config/testPass.xml
 *   2.) Start yellow (pass) team: ./soccer -y -ng -c ../config/sim.xml
 *   3.) Start blue team: ./soccer -b -c ../config/sim.xml
 *   4.) Select yellow "TestPassExperiment" and press "force start"
 *   5.) Select blue play (offense + defense + goalie) and press "force start"
 *
 *  Created on: Feb 21st, 2010
 *      Author: Philip Rogers
 *      Author:
 */

#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/OptimizedPassing.hpp>
#include <gameplay/behaviors/positions/Forward.hpp>

using namespace std;

namespace Gameplay{
	namespace Plays{
		class TestPassExperiment1: public Play{
			public:
			TestPassExperiment1(GameplayModule *gameplay);

				virtual bool applicable(){return true;}

				virtual bool run();

			protected:
				Behaviors::OptimizedPassing passPlanner_;
				Behaviors::Forward _kicker1, _kicker2;
				long _startTime;
				long _expTime;

				enum State{Initializing,WaitOppStart,Running,Done};
				State _expState;
				enum Type{TwoOffense,Pass,OptimizedPass};
				Type _expType;
		};
	}
}
