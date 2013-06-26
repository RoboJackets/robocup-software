/*
 * ReceivePointEvaluator.cpp
 *
 *  Created on: Jun 20, 2013
 *      Author: matt
 */

#include "ReceivePointEvaluator.hpp"
#include "GameplayModule.hpp"
#include "../../common/Constants.hpp"

using namespace Geometry2d;
using namespace std;

namespace Gameplay {
REGISTER_CONFIGURABLE(ReceivePointEvaluator)

ConfigBool *ReceivePointEvaluator::_visualize;

void ReceivePointEvaluator::createConfiguration(Configuration *cfg)
{
	_visualize = new ConfigBool(cfg, "ReceivePointEvaulator/Visualize", false);
}

Point ReceivePointEvaluator::FindReceivingPoint(SystemState* state, Point receiverPos, Point ballPos, Segment receivingLine, float* out_GoalWindowWidth)
{
	Segment goalSegment = Segment(Point(-Field_GoalWidth/2.0,6.05), Point(Field_GoalWidth/2.0,6.05));

	/*Point A(1.0125, 0);
	Point B(1.0125, 5.05);

	Segment line = Segment(A, B);*/
	Segment line = receivingLine;

	WindowEvaluator windower(state);
	windower.debug = (*_visualize);
	windower.clear();
	windower.exclude.clear();
	windower.exclude.push_back(receiverPos);

	windower.run(ballPos, line);

	list<Window*> windows = windower.windows;

	double longestLength = -1;
	int bestInd = 0;

	float* scores = new float[windows.size()];
	list<Window*>::iterator iter = windows.begin();
	for(int i = 0; i < windows.size(); i++)
	{
		Window* window = (*iter);
		iter++;
		WindowEvaluator WE(state);
		Point center = window->segment.center();
		WE.debug = (*_visualize);
		WE.clear();
		WE.exclude.clear();
		WE.exclude.push_back(receiverPos);
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
	Segment bestCandidate;
	for(int i = 0; i < windows.size(); i++)
	{
		if(scores[i] == longestLength)
		{
			Segment candidate = (*iter)->segment;
			Point center = candidate.center();
			float dist = receiverPos.distTo(center);//candidate.center().distTo(receiverPos);
			if(bestScore == -1 || dist < bestScore)
			{
				bestScore = dist;
				bestCandidate = candidate;
			}
		}
		iter++;
	}

	if((*_visualize))
		state->drawCircle(bestCandidate.center(), Robot_Radius, QColor(255,0,0), "ReceivePointEval");

	if(out_GoalWindowWidth)
		(*out_GoalWindowWidth) = bestCandidate.length();

	return bestCandidate.center();
}

} /* namespace Gameplay */
