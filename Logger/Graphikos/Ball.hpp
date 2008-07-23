#ifndef _GRAPHIKOS_BALL_H_
#define _GRAPHIKOS_BALL_H_

#include <Geometry/Point2d.hpp>

#include <QPainter>

namespace Graphikos
{
	/**
	 * The ball class is a graphical representation of the 
	 * golf ball
	 */
	class Ball
	{
		public:
			static void paint(QPainter& painter, float x, float y);
			
			static void paint(QPainter& painter, Geometry::Point2d pos);
			
		private:
			Ball();
			Ball(Ball&);
	};
};

#endif /*BALL_H_*/
