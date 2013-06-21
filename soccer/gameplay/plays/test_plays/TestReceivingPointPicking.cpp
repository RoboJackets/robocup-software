/*
 * TestPivotKick.cpp
 *
 *  Created on: Nov 11, 2012
 *      Author: Matthew Barulic
 */

#include "TestReceivingPointPicking.hpp"
#include <boost/foreach.hpp>
#include <iterator>

#include <QtGui>

using namespace std;
using namespace Geometry2d;

namespace Gameplay {
namespace Plays {

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestReceivingPointPicking, "TestReceivingPointPicker")

TestReceivingPointPicking::TestReceivingPointPicking(GameplayModule *gameplay):
	Play(gameplay),
	_gameplay(gameplay),
	_move(gameplay)
{
}

bool TestReceivingPointPicking::run() {
	std::cout << "get robots" << std::endl;
	set<OurRobot *> available = _gameplay->playRobots();
	std::cout << "assign robot" << std::endl;
	assignNearest(_move.robot, available, Geometry2d::Point());

	if(_move.robot)
	{
		std::cout << "got a robot" << std::endl;
		Point A(1.0125, 0);
		Point B(1.0125, 5.05);
		Segment line = Segment(A, B);
		Point reveivePoint = FindReceivingPoint(_gameplay->state(), (Robot*)_move.robot, ball().pos, line);
		std::cout << "got a point" << std::endl;
		state()->drawCircle(reveivePoint, Robot_Radius, QColor(255,0,0), "Receiver");
		std::cout << "drew the point" << std::endl;
	}

	return true;
}

Geometry2d::Point TestReceivingPointPicking::FindReceivingPoint(SystemState* state, Robot* robot, Point ballPos, Segment receivingLine)
{
	Segment goalSegment = Segment(Point(-Field_GoalWidth/2.0,6.05), Point(Field_GoalWidth/2.0,6.05));

	/*Point A(1.0125, 0);
	Point B(1.0125, 5.05);

	Segment line = Segment(A, B);*/
	Segment line = receivingLine;

	WindowEvaluator windower(state);
	windower.debug = true;
	windower.clear();
	windower.exclude.clear();
	windower.exclude.push_back(robot->pos);

	windower.run(ballPos, line);

	list<Window*> windows = windower.windows;

	double longestLength = -1;
	int bestInd = 0;

	int dColor = (int)(255/windows.size());

	float* scores = new float[windows.size()];
	list<Window*>::iterator iter = windows.begin();
	for(int i = 0; i < windows.size(); i++)
	{
		Window* window = (*iter);
		iter++;
		WindowEvaluator WE(_gameplay->state());
		Point center = window->segment.center();
		WE.debug = true;
		WE.clear();
		WE.exclude.clear();
		WE.exclude.push_back(robot->pos);
		WE.run(center, goalSegment);
		if(WE.windows.size() > 0)
		{
			scores[i] = WE.best()->segment.length();//abs(WE.best()->t1-WE.best()->t0);
			std::cout << i << " : " << scores[i] << std::endl;
			if(longestLength == -1 || scores[i] > longestLength)
			{
				longestLength = scores[i];
				bestInd = i;
			}
		}
		else
		{
			scores[i] = 0;
		}
	}

	iter = windows.begin();
	float bestScore = -1;
	Point bestCandidate;
	for(int i = 0; i < windows.size(); i++)
	{
		if(scores[i] == longestLength)
		{
			Point candidate = (*iter)->segment.center();
			float dist = candidate.distTo(robot->pos);
			if(bestScore == -1 || dist < bestScore)
			{
				bestScore = dist;
				bestCandidate = candidate;
			}
		}
		iter++;
	}

	return bestCandidate;
}

TestReceivingPointPicking::~TestReceivingPointPicking() {
}

} /* namespace Plays */
} /* namespace Gameplay */
