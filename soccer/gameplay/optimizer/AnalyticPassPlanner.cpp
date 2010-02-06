/*
 * AnalyticPassPlanner.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#include <AnalyticPassPlanner.hpp>

namespace AnalyticPassPlanner {
	void generateAllConfigs(const Point &ballPos, set<Robot *> &robots, PassConfigVector &passConfigResult){
		Geometry2d::Point goalBallPos = Geometry2d::Point(0.0, Constants::Field::Length);

		BOOST_FOREACH(Robot *r1, robots){
/*
			float deg0accel = r1->packet()->config.motion.deg0.acceleration;
			float deg0dccel = r1->packet()->config.motion.deg0.deceleration;
			float deg0vel = r1->packet()->config.motion.deg0.velocity;
			float deg45accel = r1->packet()->config.motion.deg45.acceleration;
			float deg45dccel = r1->packet()->config.motion.deg45.deceleration;
			float deg45vel = r1->packet()->config.motion.deg45.velocity;
			float raccel = r1->packet()->config.motion.rotation.acceleration;
			float rdccel = r1->packet()->config.motion.rotation.deceleration;
			float rvel = r1->packet()->config.motion.rotation.velocity;
			r1->packet()->config.motion.angle.

			cout << "deg0accel: " << deg0accel << endl;
			cout << "deg0dccel: " << deg0dccel << endl;
			cout << "deg0vel: " << deg0vel << endl;
			cout << "deg45accel: " << deg45accel << endl;
			cout << "deg45dccel: " << deg45dccel << endl;
			cout << "deg45vel: " << deg45vel << endl;
			cout << "raccel: " << raccel << endl;
			cout << "rdccel: " << rdccel << endl;
			cout << "rvel: " << rvel << endl;*/

			BOOST_FOREACH(Robot *r2, robots){
				if(r2->id()==r1->id()){continue;} // don't pass to self

				PassConfig* passConfig = new PassConfig();

				// setup calculations
				Point passVec = (r2->pos() - ballPos).normalized();
				float passAngle = passVec.angle();
				Point goalVec = (goalBallPos - r2->pos()).normalized();
				float goalAngle = goalVec.angle();

				// add initial state with no robots having the ball
				passConfig->addPassState(
						PassState(r1, r2, r1->pos(), r2->pos(), r1->angle(), r2->angle(),
						ballPos, PassState::INTERMEDIATE, 0));

				// add state with robot1 at a position to pass to robot2
				Point state2Robot1Pos = ballPos - passVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
				float state2Robot1Rot = passAngle;
				double state2Time = r1->pos().distTo(state2Robot1Pos) / APPROXROBOTVELTRANS;
				passConfig->addPassState(
						PassState(r1, r2, state2Robot1Pos, r2->pos(), state2Robot1Rot, r2->angle(),
						ballPos, PassState::KICKPASS, state2Time));

				// add state with robot2 receiving ball
				Point state3BallPos = r2->pos();
				Point state3Robot2Pos = state3BallPos + passVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
				float state3Robot2Rot = (passVec * -1.0f).angle();
				double state3Time = state2Time + r2->pos().distTo(state3Robot2Pos) / APPROXROBOTVELTRANS;
				passConfig->addPassState(
						PassState(r1, r2, state2Robot1Pos, state3Robot2Pos, state2Robot1Rot, state3Robot2Rot,
						state3BallPos, PassState::RECEIVEPASS, state3Time));

				// add state with robot2 kicking a goal
				Point state4Robot2Pos = state3BallPos - goalVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
				float state4Robot2Rot = goalAngle;
				double state4Time = state3Time + state3Robot2Pos.distTo(state4Robot2Pos) / APPROXROBOTVELTRANS;
				passConfig->addPassState(
						PassState(r1, r2, state2Robot1Pos, state4Robot2Pos, state2Robot1Rot, state4Robot2Rot,
						state3BallPos, PassState::KICKGOAL, state4Time));

				// add state with ball in goal
				double state5Time = state4Time + state3BallPos.distTo(goalBallPos) / APPROXBALLVEL;
				passConfig->addPassState(
						PassState(r1, r2, state2Robot1Pos, state4Robot2Pos, state2Robot1Rot, state4Robot2Rot,
						goalBallPos, PassState::INTERMEDIATE, state5Time));

				passConfigResult.push_back(passConfig);
			}
		}
	}

	void evaluateConfigs(set<Robot *> &robots, Robot** opponents, PassConfigVector &passConfigs){
		//
		// Weight configs
		//

		PassState prevState;

		for(int i=0; i<(int)passConfigs.size(); i++){
			int numInteractions = 0;

			// calculate the total number of opponents that can touch the ball at each intermediate
			// ballPos (where the ball will be waiting for the robot to pivot and pass)
			for(int j=0; j<passConfigs[i].length(); j++){
				PassState thisState = passConfigs[i].getPassState(j);
				for (int i=0; i<Constants::Robots_Per_Team; ++i){
					Robot *opponentR = opponents[i];
					double robotTravelDist = opponentR->pos().distTo(thisState.ballPos);
					double robotTravelTime = robotTravelDist / APPROXROBOTVELTRANS;
					if(robotTravelTime < thisState.timestamp){
						numInteractions++;
					}
				}
			}

			// calculate the total number of opponents that currently intersect
			// the ball's path at this instant.
			for(int j=0; j<passConfigs[i].length(); j++){
				PassState thisState = passConfigs[i].getPassState(j);
				Line ballPath(thisState.ballPos,prevState.ballPos);
				for (int i=0; i<Constants::Robots_Per_Team; ++i){
					Robot *opponentR = opponents[i];
					// we use 2*Radius to give "wiggle room"
					if(ballPath.distTo(opponentR->pos()) < (float)(Constants::Robot::Radius + 2*Constants::Ball::Radius)){
						numInteractions++;
					}
				}
				prevState = thisState;
			}

			passConfigs[i].setWeight(numInteractions);
		}

		passConfigs.sort();
	}
}
