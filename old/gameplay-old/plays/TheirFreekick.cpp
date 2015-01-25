#include "TheirFreekick.hpp"

#include <iostream>
#include <limits>


using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TheirFreekick, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(TheirFreekick)
	}
}

void Gameplay::Plays::TheirFreekick::createConfiguration(Configuration *cfg)
{

}

Gameplay::Plays::TheirFreekick::TheirFreekick(GameplayModule *gameplay):
	Play(gameplay),
	_defender1(gameplay, Behaviors::Defender::Left),
	_defender2(gameplay, Behaviors::Defender::Right),
	_marking1(gameplay),
	_marking2(gameplay),
	_marking3(gameplay)
{
	_defender1.otherDefenders.insert(&_defender2);
	_defender2.otherDefenders.insert(&_defender1);

	// assign general parameters
	float r = 0.7;
	_marking1.ratio(r);
	_marking2.ratio(r);
	_marking3.ratio(r);
}

float Gameplay::Plays::TheirFreekick::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.theirFreeKick()) ? 0 : INFINITY;
}

bool Gameplay::Plays::TheirFreekick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	assignNearest(_defender1.robot, available, Geometry2d::Point(-Field_GoalWidth/2, 0.0));
	assignNearest(_defender2.robot, available, Geometry2d::Point( Field_GoalWidth/2, 0.0));

	//FIXME - How to choose?
	assignNearest(_marking1.robot, available, Geometry2d::Point());
	assignNearest(_marking2.robot, available, Geometry2d::Point());
	assignNearest(_marking3.robot, available, Geometry2d::Point());
	
	if (!ball().valid)
	{
		return false;
	}
	Point ballPos = ball().pos;

	//  determine which robots to mark
	map<float, OpponentRobot*> open_opp;
	OpponentRobot* closestRobot = NULL;
	float closestRobotDist = numeric_limits<float>::infinity();
	float closestOpenDist = numeric_limits<float>::infinity();
	for (OpponentRobot * r :  state()->opp) {
		if (r && r->visible) {
			// want maximum distance that is less than 3 meters from any of our robots
			Point oppPos = r->pos;
			float ballDist = oppPos.distTo(ballPos);
			float max_relevant_robot_range = 3.0; // meters
			float closestSelfDist = 100.0;
			if (ballDist < max_relevant_robot_range) {
				for (OurRobot * s :  state()->self) {
					float selfDist = s->pos.distTo(oppPos);
					if (selfDist < closestSelfDist) {
						closestSelfDist = selfDist;
					}
				}
				if (closestSelfDist < 100.0) {
					// record the robot with distance, sort by open-ness
					open_opp[closestSelfDist] = r;
				}
			}

			if (ballDist < closestRobotDist) {
				closestRobot = r;
				closestRobotDist = ballDist;
				closestOpenDist = closestSelfDist;
			}
		}
	}

	// remove the closest robot to the ball, as it is actually kicking
	if (closestRobot) {
		open_opp.erase(closestOpenDist);
	}

	// assign targets
	if (open_opp.empty()) {
		// if nothing open, just drive near the ball, but not within radius
		if (_marking1.robot)
		{
			Geometry2d::Point pos = _marking1.robot->pos;
			_marking1.robot->move(pos + (ballPos-pos).normalized() * (ballPos.distTo(pos) - Field_CenterRadius));
			_marking1.robot->face(ballPos);
		}
		if (_marking2.robot)
		{
			Geometry2d::Point pos = _marking2.robot->pos;
			_marking2.robot->move(pos + (ballPos-pos).normalized() * (ballPos.distTo(pos) - Field_CenterRadius));
			_marking2.robot->face(ballPos);
		}
		if (_marking3.robot)
		{
			Geometry2d::Point pos = _marking3.robot->pos;
			_marking3.robot->move(pos + (ballPos-pos).normalized() * (ballPos.distTo(pos) - Field_CenterRadius));
			_marking3.robot->face(ballPos);
		}
	} else if (open_opp.size() == 1) {
		if (_marking1.robot)
		{
			_marking1.markRobot(open_opp.begin()->second);
			_marking1.run();
		}
		if (_marking2.robot)
		{
			Geometry2d::Point pos = _marking2.robot->pos;
			_marking2.robot->move(pos + (ballPos-pos).normalized() * (ballPos.distTo(pos) - Field_CenterRadius));
			_marking2.robot->face(ballPos);
		}
		if (_marking3.robot)
		{
			Geometry2d::Point pos = _marking3.robot->pos;
			_marking3.robot->move(pos + (ballPos-pos).normalized() * (ballPos.distTo(pos) - Field_CenterRadius));
			_marking3.robot->face(ballPos);
		}
	} else if (open_opp.size() == 2) {
		if (_marking1.robot)
		{
			_marking1.markRobot(open_opp.begin()->second);
			_marking1.run();
		}
		if (_marking2.robot)
		{
			_marking2.markRobot((++open_opp.begin())->second);
			_marking2.run();
		}
		if (_marking3.robot)
		{
			Geometry2d::Point pos = _marking3.robot->pos;
			_marking3.robot->move(pos + (ballPos-pos).normalized() * (ballPos.distTo(pos) - Field_CenterRadius));
			_marking3.robot->face(ballPos);
		}
	} else {
		if (_marking1.robot)
		{
			_marking1.markRobot(open_opp.begin()->second);
			_marking1.run();
		}
		if (_marking2.robot)
		{
			_marking2.markRobot((++open_opp.begin())->second);
			_marking2.run();
		}
		if (_marking3.robot)
		{
			_marking3.markRobot((++(++open_opp.begin()))->second);
			_marking3.run();
		}
	}

	// adjust obstacles on markers
	if (_marking1.robot && _marking2.robot) {
		unsigned m1 = _marking1.robot->shell(), m2 = _marking2.robot->shell();
		_marking1.robot->avoidTeammateRadius(m2, 0.5);
		_marking2.robot->avoidTeammateRadius(m1, 0.5);
		if (_marking3.robot) {
			unsigned m3 = _marking3.robot->shell();
			_marking1.robot->avoidTeammateRadius(m3, 0.5);
			_marking2.robot->avoidTeammateRadius(m3, 0.5);
			_marking3.robot->avoidTeammateRadius(m1, 0.5);
			_marking3.robot->avoidTeammateRadius(m2, 0.5);
		}
	}

	// execute default defender "wall" behavior
	_defender1.run();
	_defender2.run();
	
	return true;
}
