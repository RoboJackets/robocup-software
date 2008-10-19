#ifndef _xisop_circle2d_h
#define _xisop_circle2d_h

#include "Point2d.hpp"

namespace Geometry
{
	/**
	class for a circle in 2d space
	*/
	class Circle2d
	{
		public:
			/**
			default constructor
			*/
			Circle2d();
			
			/**
			constructor for x, y, and radius
			@param x the x coordinate
			@param y the y coordinate
			@param radius the radius of the circle
			*/
			Circle2d(float x, float y, float radius);
			Circle2d(Geometry::Point2d pos, float radius);
			~Circle2d();
			
			/** get the x coordinate */
			float x() const { return _pos.x; }
			/** get the y coordinate */
			float y() const { return _pos.y; }
			/** get the radius */
			float radius() const;
			/** return the center of the circle */
			Point2d center() const { return _pos; }
			
			/** set the x coordinate */
			void setX(float x) { _pos.x = x; }
			/** set the y coordiante */
			void setY(float y) { _pos.y = y; }
			/** set both coordinates at the same time */
			void setXY(float x, float y);
			/** sets the radius */
			void setRadius(float radius);
			
			bool tangentPoints(const Geometry::Point2d src, 
					Geometry::Point2d* p1 = 0, Geometry::Point2d* p2 = 0) const;
		protected:
			Point2d _pos;
			
			/** the radius */
			float _radius;
	};
};

#endif //_circle2d_h
