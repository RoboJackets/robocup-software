#ifndef _GRAPHIKOS_ROBOT_H_
#define _GRAPHIKOS_ROBOT_H_

#include <Team.h>
#include <Geometry/Point2d.hpp>

#include <QPainter>

namespace Graphikos
{
	/**
	 * Graphical representation of a robot
	 */
	class Robot
	{
		public:	
			static void paint(QPainter& painter, Team t, unsigned char ID, float x, float y, float theta);
			
			static void paint(QPainter& painter, Team t, unsigned char ID, Geometry::Point2d pos, float theta);
			
		private:
			Robot();
			Robot(Robot&);
			
	};
};

#endif
