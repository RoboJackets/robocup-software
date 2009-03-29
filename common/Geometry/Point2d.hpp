#ifndef _XISOP_GEOMETRY_POINT_HPP
#define _XISOP_GEOMETRY_POINT_HPP

#include <math.h>

namespace Geometry
{
	/**
	Simple class to represent a point in 2d space. Uses floating point coordinates
	*/
	class Point2d
	{
		public:
			/**
			default constrctor.
			initializes point to (0,0)
			*/
			Point2d()
			{
				x = y = 0;
			}

			/**
			overloaded constructor.
			sets the point to x,y
			@param x the x coordinate
			@param y the y coordinate
			*/
			Point2d(float x, float y)
			{
				this->x = x;
				this->y = y;
			}

			Point2d operator+(Point2d other) const
			{
				return Point2d(x + other.x, y + other.y);
			}

			Point2d operator-(Point2d other) const
			{
				return Point2d(x - other.x, y - other.y);
			}

			Point2d operator-() const
			{
				return Point2d(-x, -y);
			}

			Point2d &operator+=(Point2d other)
			{
				x += other.x;
				y += other.y;

				return *this;
			}

			Point2d &operator-=(Point2d other)
			{
				x -= other.x;
				y -= other.y;

				return *this;
			}

			Point2d &operator*=(float s)
			{
				x *= s;
				y *= s;

				return *this;
			}

			Point2d &operator/=(float s)
			{
				x /= s;
				y /= s;

				return *this;
			}

			Point2d operator/(float s) const
			{
				return Point2d(x / s, y / s);
			}

			Point2d operator*(float s) const
			{
				return Point2d(x * s, y * s);
			}

			bool operator==(Point2d other) const
			{
				return x == other.x && y == other.y;
			}
			
			bool operator!=(Point2d other) const
			{
				return !((*this) == other);
			}
			
			/**
			computes the dot product of this point and another.
			behaves as if the poitns were 2d vectors
			@param p the second point
			@return the dot product of the two
			*/
			float dot(Point2d p) const
			{
				return x * p.x + y * p.y;
			}
			
			/**
			find out of the point is 0,0
			@return true if the point is (0,0)
			*/
			bool isZero() const
			{
				return x == 0 && y == 0;
			}
			
			/**
			computes the magnitude of the point, as if it were a vector
			@return the magnitude of the point
			*/
			float mag() const
			{
				return sqrtf(x * x + y * y);
			}
			
			/**
			computes magnitude squared
			@return the magnitude squared
			*/
			float magsq() const
			{
				return x * x + y * y;
			}
			
			/**
			rotates the point around another point by apecified angle in the CCW direction
			@param origin the point to rotate around
			@param angle the angle in degrees
			*/
			void rotate(const Point2d &origin, float angle);

			/**
			computes the distance from the current point to another
			@param other the point to find the distance to
			@return the distance between the points
			*/
			float distTo(const Point2d &other) const
			{
				Point2d delta = other - *this;
				return delta.mag();
			}
			
			/**
			 * Returns a vector with the same direction as this vector but with magnitude one,
             * unless this vector is zero.
             */
			Point2d norm() const
			{
                float m = mag();
				if (m == 0)
				{
					return Point2d(0, 0);
				}
				
				return Point2d(x / m , y / m);
			}
			
			/**
             * Returns true if this point is within the given distance (threshold) of (pt)
			 */
			bool inThreshold(const Point2d &pt, float threshold) const;
			
			/**
			 * Returns the angle of this point in radians CCW from +X.
			 */
			float angle() const
			{
				return atan2(y, x);
			}
			
            /**
             * Returns a unit vector in the given direction (in radians)
             */
            static Point2d direction(float theta)
            {
                return Point2d(cos(theta), sin(theta));
            }
            
            /** returns the perpendicular to the point, Clockwise */
            Point2d perpCW() const
            {
            	return Point2d(y, -x);
            }
            
            /** returns the perpendicular to the point, Counter Clockwise */
            Point2d perpCCW() const
            {
            	return Point2d(-y, x);
            }
            
			/** the x coordinate */
			float x;
			
			/** the y coordinate */
			float y;
	};
};

#endif
