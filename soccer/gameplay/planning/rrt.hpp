// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <list>
#include <Geometry2d/Point.hpp>
#include <framework/Obstacle.hpp>
#include <framework/Path.hpp>

#include "Tree.hpp"

namespace Planning
{
	namespace RRT
	{
		/** generate a random point on the floor */
		Geometry2d::Point randomPoint();
		/**
		 * RRT: http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
		 * this plans the motion path for the robot
		 */
		class Planner
		{
			public:
				Planner();
				/**
				 * gets the maximum number of iterations for the RRT algorithm
				 */
				int maxIterations() const
				{
					return _maxIterations;
				}
				/**
				 * sets the maximum number of iterations for th RRT algorithm
				 */
				void maxIterations(int value)
				{
					_maxIterations = value;
				}
				
				///run the path planner
				///this will always populate path to be the path we need to travel
				void run(
						const Geometry2d::Point& start,
						const float angle, 
						const Geometry2d::Point& vel, 
						const Geometry2d::Point& goal, 
						const ObstacleGroup* obstacles, 
						Planning::Path &path);
				
				/** returns the length of the best position planned path */
				float fixedPathLength() const { return _bestPath.length(); }
				
		protected:
			FixedStepTree _fixedStepTree0;
			FixedStepTree _fixedStepTree1;
			
			/** best goal point */
			Geometry2d::Point _bestGoal;
			
			///best planned path
			///this is a fixed step path
			Planning::Path _bestPath;
			
			///maximum number of rrt iterations to run
			///this does not include connect attempts
			unsigned int _maxIterations;
			
			///latest obstacles
			const ObstacleGroup* _obstacles;
			
			/** makes a path from the last point of each tree
			 *  If the points don't match up...fail!
			 *  The final path will be from the start of tree0
			 *  to the start of tree1 */
			void makePath();
			
			/** optimize the path */
			void optimize(Planning::Path &path, const ObstacleGroup *obstacles);
		};
	}
}
