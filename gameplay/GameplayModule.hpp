// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QMutex>

#include <framework/Module.hpp>

#include "Playbook.hpp"
#include "Robot.hpp"

namespace Gameplay
{
	class WindowEvaluator;
	
	class GameplayModule: public Module
	{
		public:
			GameplayModule();
			
			void createGoalie();
			void loadPlays(const char *dir);
			void loadPlay(const char* path);
			
			virtual void fieldOverlay(QPainter&, Packet::LogFrame&) const;
			virtual void run();
			virtual void mousePress(QMouseEvent* me, Geometry2d::Point pos);
			virtual void mouseMove(QMouseEvent* me, Geometry2d::Point pos);
			virtual void mouseRelease(QMouseEvent* me, Geometry2d::Point pos);
			
			// Gameplay robots
			Robot *self[Constants::Robots_Per_Team];
			Robot *opp[Constants::Robots_Per_Team];
			
			// Finds all robots running a behavior of a given type
			// (or descended from that type).
			template<class T>
			void find_by_type(std::list<Robot *> &robots)
			{
				for (int i = 0; i < Constants::Robots_Per_Team; ++i)
				{
					Robot *r = self[i];
					Behavior *behavior = r->behavior();
					if (behavior && dynamic_cast<T>(behavior))
					{
						robots.push_back(r);
					}
				}
			}
			
			template<class T>
			void find_not_of_type(std::list<Robot *> &robots)
			{
				for (int i = 0; i < Constants::Robots_Per_Team; ++i)
				{
					Robot *r = self[i];
					Behavior *behavior = r->behavior();
					if (behavior && !dynamic_cast<T>(behavior))
					{
						robots.push_back(r);
					}
				}
			}

			// Finds a robot by name.
			// Searches self robots first, then opponents.
			Robot *find(const std::string &name);

			int availableRobots() const
			{
				return _availableRobots;
			}
			
			Playbook playbook;
			
		protected:
			ObstaclePtr _sideObstacle;
			
			//outside of the floor boundaries
			ObstaclePtr _nonFloor[4];
			
			//goal area
			ObstaclePtr _goalArea[3];
			
			// Number of assignable robots
			int _availableRobots;
	};
}
