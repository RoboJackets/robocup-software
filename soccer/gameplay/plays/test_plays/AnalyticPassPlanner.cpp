/*
 * AnalyticPassPlanner.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#include <gameplay/plays/test_plays/AnalyticPassPlanner.hpp>

namespace AnalyticPassPlanner {
	void generateAllConfigs(const Point &ballPos, set<Robot *> &robots, PassConfigVector &passConfigResult){
		Geometry2d::Point goalBallPos = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length);

		BOOST_FOREACH(Robot *r1, robots){
			// initialize configurations of length 1
			PassConfig* passConfig = new PassConfig(); // add starting ball position
			passConfig->addPassState(new PassState(ballPos,PassState::INITIAL)); // add robot with ball
			passConfig->addPassState(new PassState(ballPos, r1)); // add ending (goal) ball position
			passConfig->addPassState(new PassState(goalBallPos,PassState::GOAL));
			passConfigResult.push_back(passConfig);

			// initialize configurations of length 2
			BOOST_FOREACH(Robot *r2, robots){
				if(r2->id()==r1->id()){continue;} // don't pass to self
				PassConfig* passConfig = new PassConfig(); // add starting ball position
				passConfig->addPassState(new PassState(ballPos,PassState::INITIAL)); // add robot1 with ball
				passConfig->addPassState(new PassState(ballPos, r1)); // add robot2 with ball
				passConfig->addPassState(new PassState(r2->pos(), r2)); // add ending (goal) ball position
				passConfig->addPassState(new PassState(goalBallPos,PassState::GOAL));
				passConfigResult.push_back(passConfig);

				/*
				// initialize configurations of length 3
				BOOST_FOREACH(Robot *r3, robots){
					if(r3->id()==r2->id()){continue;} // don't pass to self
													  // note: r1 -> r2 -> r1 is ok.
					PassConfig* passConfig = new PassConfig(); // add starting ball position
					passConfig->addPassState(new PassState(ballPos,PassState::INITIAL)); // add robot1 with ball
					passConfig->addPassState(new PassState(ballPos, r1)); // add robot2 with ball
					passConfig->addPassState(new PassState(r2->pos(), r2)); // add robot3 with ball
					passConfig->addPassState(new PassState(r3->pos(), r3)); // add ending (goal) ball position
					passConfig->addPassState(new PassState(goalBallPos,PassState::GOAL));
					passConfigResult(passConfig);
				}
				*/
			}
		}
	}

	void evaluateConfigs(set<Robot *> &robots, Robot** opponents, PassConfigVector &passConfigs){
		//
		// Weight configs
		//
		PassState *prevState = (PassState*)NULL, *thisState, *nextState;
		float ballTravelDist, robotTravelDist, robotRotateDist;
		double ballTravelTime, robotTravelTime, robotRotateTime;
		Point inVector, outVector;

		for(int i=0; i<(int)passConfigs.size(); i++){
			// calculate the timestamps at each state in the config
			//
			for(int j=0; j<passConfigs[i].length(); j++){
				thisState = passConfigs[i].getPassState(j);
				nextState = (j+1<passConfigs[i].length() ? passConfigs[i].getPassState(j+1) : thisState);
				switch(thisState->stateType){
					case PassState::INITIAL :
						thisState->timeEnterState = 0.0;
						thisState->timeLeaveState = 0.0;
						break;
					case PassState::INTERMEDIATE :
						if(prevState->stateType == PassState::INITIAL){
							robotTravelDist = thisState->robot->pos().distTo(thisState->ballPos);
							robotTravelTime = robotTravelDist / APPROXROBOTVELTRANS;
							thisState->timeEnterState = prevState->timeLeaveState + robotTravelTime;
						}else{
							ballTravelDist = thisState->ballPos.distTo(prevState->ballPos);
							ballTravelTime = ballTravelDist / APPROXBALLVEL;
							thisState->timeEnterState = prevState->timeLeaveState + ballTravelTime;
						}
						inVector = prevState->ballPos - thisState->ballPos;
						outVector = nextState->ballPos - thisState->ballPos;
						robotRotateDist = inVector.angleTo(outVector);
						robotRotateTime = robotRotateDist / APPROXROBOTVELROT;
						thisState->timeLeaveState = thisState->timeEnterState + robotRotateTime;
						break;
					case PassState::GOAL :
						ballTravelDist = thisState->ballPos.distTo(prevState->ballPos);
						ballTravelTime = ballTravelDist / APPROXBALLVEL;
						thisState->timeEnterState = prevState->timeLeaveState + ballTravelTime;
						thisState->timeLeaveState = thisState->timeEnterState;
						break;
				}
				prevState = thisState;
			}

			// calculate the total number of opponents that can touch the ball along the path
			//
			int numInteractions = 0;
			for(int j=0; j<passConfigs[i].length(); j++){
				thisState = passConfigs[i].getPassState(j);
				for (int i=0; i<Constants::Robots_Per_Team; ++i)
				{
					Robot *opponentR = opponents[i];
					robotTravelDist = opponentR->pos().distTo(thisState->ballPos);
					robotTravelTime = robotTravelDist / APPROXROBOTVELTRANS;
					if(robotTravelTime < thisState->timeLeaveState){
						numInteractions++;
					}
				}
			}

			passConfigs[i].setWeight(numInteractions);
		}
		passConfigs.sort();
	}

}
