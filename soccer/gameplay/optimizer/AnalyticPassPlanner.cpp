/*
 * AnalyticPassPlanner.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#include <gameplay/optimizer/AnalyticPassPlanner.hpp>
#include <motion/planning/rrt.hpp>
#include <framework/Path.hpp>
#include <gameplay/optimizer/PassState.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

using namespace Geometry2d;
using namespace Gameplay;
using namespace std;

#define TIME_TO_AIM_APPROX 0.3 // seconds it takes for the robot to pivot for an aim. Due to aiming time in kick behavior, even if we knew rot accel, this is an approximation.
#define BALL_KICK_AVG_VEL 1.0 // average speed of ball during a kick ((kick vel + end vel) / 2) very conservative due to inaccurate kick speeds and varying dynamics

void AnalyticPassPlanner::generateAllConfigs(const Point &ballPos, set<Robot *> &robots, PassConfigVector &passConfigResult){
	Geometry2d::Point goalBallPos = Geometry2d::Point(0.0, Constants::Field::Length);

	Planning::Path path;
	ObstacleGroup og;
	float pathDist, pathTime;

	// for times and distances, use a conservative estimate of 45deg. travel
	Robot* rTmp = *(robots.begin());
	float maxVel = rTmp->packet()->config.motion.deg45.velocity;
	float timeStopToMaxVel = maxVel/ rTmp->packet()->config.motion.deg45.acceleration;
	float timeMaxVelToStop = maxVel / rTmp->packet()->config.motion.deg45.deceleration;
	float distStopToMaxVel = 0.5 * maxVel * timeStopToMaxVel;
	float distMaxVelToStop = 0.5 * maxVel * timeMaxVelToStop;

	BOOST_FOREACH(Robot *r1, robots){
		if(!r1->visible()){continue;} // don't use invisible robots

		BOOST_FOREACH(Robot *r2, robots){
			if(!r2->visible()){continue;} // don't use invisible robots
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
							ballPos, PassState::INITIAL, 0));

			// add state with robot1 at a position to pass to robot2
			Point state2Robot1Pos = ballPos - passVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
			float state2Robot1Rot = passAngle;
			Point state2Robot2Pos = r2->pos();
			float state2Robot2Rot = (passVec * -1.0f).angle();

			// calculate time
			og = r1->obstacles();
			Point r1pos = r1->pos();
			float r1angle = r1->angle();
			Point r1vel = r1->vel();
			Motion::RRT::Planner planner;
			Motion::Dynamics dynamics;
			dynamics.setConfig(_gameplay->state()->self[r1->id()].config.motion);
			planner.setDynamics(&dynamics);
			planner.run(r1pos,r1angle,r1vel,ballPos,&og,path);
			pathDist = path.length(0);
			if(pathDist < distStopToMaxVel + distMaxVelToStop){
				pathTime = (pathDist/(distStopToMaxVel + distMaxVelToStop))*(timeStopToMaxVel + timeMaxVelToStop);
			}else{
				pathTime = timeStopToMaxVel + timeMaxVelToStop + (pathDist - (distStopToMaxVel + distMaxVelToStop))/maxVel;
			}
			double state2Time = pathTime + TIME_TO_AIM_APPROX;

			passConfig->addPassState(
					PassState(r1, r2, state2Robot1Pos, state2Robot2Pos, state2Robot1Rot, state2Robot2Rot,
							ballPos, PassState::KICKPASS, state2Time));

			// add state with robot2 receiving ball
			Point state3BallPos = state2Robot2Pos;
			Point state3Robot2Pos = state3BallPos + passVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
			float state3Robot2Rot = (passVec * -1.0f).angle();
			double state3Time = state2Time + r2->pos().distTo(state3Robot2Pos) / BALL_KICK_AVG_VEL;
			passConfig->addPassState(
					PassState(r1, r2, state2Robot1Pos, state3Robot2Pos, state2Robot1Rot, state3Robot2Rot,
							state3BallPos, PassState::RECEIVEPASS, state3Time));

			// add state with robot2 kicking a goal
			Point state4Robot2Pos = state3BallPos - goalVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
			float state4Robot2Rot = goalAngle;
			double state4Time = state3Time + state3Robot2Pos.distTo(state4Robot2Pos) / TIME_TO_AIM_APPROX;
			passConfig->addPassState(
					PassState(r1, r2, state2Robot1Pos, state4Robot2Pos, state2Robot1Rot, state4Robot2Rot,
							state3BallPos, PassState::KICKGOAL, state4Time));

			// add state with ball in goal
			double state5Time = state4Time + state3BallPos.distTo(goalBallPos) / BALL_KICK_AVG_VEL;
			passConfig->addPassState(
					PassState(r1, r2, state2Robot1Pos, state4Robot2Pos, state2Robot1Rot, state4Robot2Rot,
							goalBallPos, PassState::GOAL, state5Time));

			passConfigResult.push_back(passConfig);
		}
	}
}

void AnalyticPassPlanner::evaluateConfigs(set<Robot *> &robots, Robot** opponents, PassConfigVector &passConfigs){
	Geometry2d::Point goalBallPos = Geometry2d::Point(0.0, Constants::Field::Length);
	//
	// Weight configs
	//
	PassState prevState;

	// locate opponent's goalie by finding closest opponent to goal
	Robot *oppGoalie = opponents[0];
	float bestDist = oppGoalie->pos().distTo(goalBallPos), thisDist;
	for (int i=0; i<Constants::Robots_Per_Team; ++i){
		Robot *opponentR = opponents[i];
		thisDist = opponentR->pos().distTo(goalBallPos);
		if(thisDist < bestDist){
			oppGoalie = opponentR;
			bestDist = thisDist;
		}
	}

	Planning::Path path;Motion::RRT::Planner planner;
	ObstacleGroup og;
	float pathDist, pathTime;

	Robot* rTmp = *(robots.begin());
	float maxVel = rTmp->packet()->config.motion.deg45.velocity;
	float timeStopToMaxVel = maxVel/ rTmp->packet()->config.motion.deg45.acceleration;
	float timeMaxVelToStop = maxVel / rTmp->packet()->config.motion.deg45.deceleration;
	float distStopToMaxVel = 0.5 * maxVel * timeStopToMaxVel;
	float distMaxVelToStop = 0.5 * maxVel * timeMaxVelToStop;

	for(int i=0; i<(int)passConfigs.size(); i++){
		int numInteractions = 0;

		// calculate the total number of opponents that can touch the ball at each intermediate
		// ballPos (where the ball will be waiting for the robot to pivot and pass)
		for(int j=0; j<passConfigs[i].length(); j++){
			PassState thisState = passConfigs[i].getPassState(j);
			for (int i=0; i<Constants::Robots_Per_Team; ++i){
				Robot *opponentR = opponents[i];
				if(opponentR->id() == oppGoalie->id()){continue;} // do not consider opponent's goalie
				og = opponentR->obstacles();
				Motion::RRT::Planner planner;
				planner.run(opponentR->pos(),opponentR->angle(),opponentR->vel(),thisState.ballPos,&og,path);
				pathDist = path.length(0);
				if(pathDist < distStopToMaxVel + distMaxVelToStop){
					pathTime = (pathDist/(distStopToMaxVel + distMaxVelToStop))*(timeStopToMaxVel + timeMaxVelToStop);
				}else{
					pathTime = timeStopToMaxVel + timeMaxVelToStop + (pathDist - (distStopToMaxVel + distMaxVelToStop))/maxVel;
				}
				if(pathTime < thisState.timestamp){
					numInteractions++;
				}
			}
			if(thisState.stateType == PassState::KICKGOAL){
				break; // do not include the last state with ball in goal.
			}
		}

		// calculate the total number of opponents that currently intersect
		// the ball's path at this instant.
		for(int j=0; j<passConfigs[i].length(); j++){
			PassState thisState = passConfigs[i].getPassState(j);
			Line ballPath(thisState.ballPos,prevState.ballPos);
			for (int i=0; i<Constants::Robots_Per_Team; ++i){
				Robot *opponentR = opponents[i];
				if(opponentR->id() == oppGoalie->id()){continue;} // do not consider opponent's goalie
				// use 1.5*Radius to give "wiggle room"
				if(ballPath.distTo(opponentR->pos()) < (float)(Constants::Robot::Radius + 1.5*Constants::Ball::Radius)){
					numInteractions++;
				}
			}
			prevState = thisState;
		}

		passConfigs[i].setWeight(numInteractions);
	}

	passConfigs.sort();
}
