#include "GraphNode.hpp"

#include <boost/foreach.hpp>

Gameplay::GraphNode::GraphNode(GameplayModule *gameplay)
{
	_gameplay = gameplay;
}

Gameplay::GraphNode *Gameplay::GraphNode::nextNode()
{
	float best = 0;
	GraphNode *bestNode = 0;
	BOOST_FOREACH(GraphNode *other, edges)
	{
		float score = other->score(this);
		if (!bestNode || score > best)
		{
			best = score;
			bestNode = other;
		}
	}

	return bestNode;
}

Gameplay::RobotNode::RobotNode(GameplayModule *gameplay)
	: GraphNode(gameplay)
{
	//robot = 0;
}

float Gameplay::RobotNode::score(GraphNode *from)
{
	RobotNode *fromRobot = dynamic_cast<RobotNode *>(from);
	if (!fromRobot)
	{
		return 0;
	}
	
	//FIXME - Size of window
	const float threshold = Constants::Robot::Radius + Constants::Ball::Radius;
	BOOST_FOREACH(Robot *r, _gameplay->opp)
	{
		if (fromRobot->robot)
		{
			Geometry2d::Segment seg(fromRobot->robot->pos(), robot->pos());
			if (seg.nearPoint(r->pos(), threshold))
			{
				// Path is blocked by an opponent
				return 0;
			}
		}
	}
	
	return 1;
}

Gameplay::GoalNode::GoalNode(GameplayModule *gameplay)
	: GraphNode(gameplay)
{
}

float Gameplay::GoalNode::score(GraphNode *from)
{
	return 0;
}
