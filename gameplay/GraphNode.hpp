#pragma once

#include <vector>
#include <Geometry2d/Point.hpp>
#include "GameplayModule.hpp"

namespace Gameplay
{
	class GraphNode
	{
		public:
			GameplayModule * gameplay() const
			{
				return _gameplay;
			}

			std::vector<GraphNode*> edges;

			/** Sets the type of node */
			GraphNode(GameplayModule *gameplay);

			/** Returns next node on best path to goal - NULL if not connected */
			GraphNode * nextNode();

			// Returns a score for a pass from the given node to this node
			virtual float score(GraphNode *from) = 0;

		protected:
			GameplayModule *_gameplay;
	};

	class RobotNode: public GraphNode
	{
		public:
			RobotNode(GameplayModule *gameplay);

			virtual float score(GraphNode *from);

			Robot *robot;
	};

	class GoalNode: public GraphNode
	{
		public:
			GoalNode(GameplayModule *gameplay);

			virtual float score(GraphNode *from);
	};
}
