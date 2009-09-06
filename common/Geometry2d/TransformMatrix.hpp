#pragma once

#include "Point.hpp"
#include "util.h"

namespace Geometry2d
{
	// A 2x3 transformation matrix.
	//
	// This is the 2D equivalent of the usual 3D tranformation matrix
	// with the bottom row omitted because the bottom (third) element
	// of a 2D point is always 1.  The third row of the matrix is
	// understood to be [0 0 1].
	class TransformMatrix
	{
		public:
			TransformMatrix()
			{
				_m[0] = 1;
				_m[1] = 0;
				_m[2] = 0;
				_m[3] = 0;
				_m[4] = 1;
				_m[5] = 0;
			}

			TransformMatrix(float a, float b, float c, float d, float e, float f)
			{
				_m[0] = a;
				_m[1] = b;
				_m[2] = c;
				_m[3] = d;
				_m[4] = e;
				_m[5] = f;
			}

			TransformMatrix(const Geometry2d::Point &origin, float rotation = 0, bool mirror = false, float scale = 1);

			TransformMatrix operator*(const TransformMatrix &other) const
			{
				float a = _m[0] * other._m[0] + _m[1] * other._m[3];
				float b = _m[0] * other._m[1] + _m[1] * other._m[4];
				float c = _m[0] * other._m[2] + _m[1] * other._m[5] + _m[2];
				float d = _m[3] * other._m[0] + _m[4] * other._m[3];
				float e = _m[3] * other._m[1] + _m[4] * other._m[4];
				float f = _m[3] * other._m[2] + _m[4] * other._m[5] + _m[5];

				return TransformMatrix(a, b, c, d, e, f);
			}

			TransformMatrix &operator*=(const TransformMatrix &other)
			{
				float a = _m[0] * other._m[0] + _m[1] * other._m[3];
				float b = _m[0] * other._m[1] + _m[1] * other._m[4];
				float c = _m[0] * other._m[2] + _m[1] * other._m[5] + _m[2];
				float d = _m[3] * other._m[0] + _m[4] * other._m[3];
				float e = _m[3] * other._m[1] + _m[4] * other._m[4];
				float f = _m[3] * other._m[2] + _m[4] * other._m[5] + _m[5];

				_m[0] = a;
				_m[1] = b;
				_m[2] = c;
				_m[3] = d;
				_m[4] = e;
				_m[5] = f;

				return *this;
			}

			Point operator*(const Point &pt) const
			{
				return Point(pt.x * _m[0] + pt.y * _m[1] + _m[2],
                               pt.x * _m[3] + pt.y * _m[4] + _m[5]);
			}
			
            // Transforms a direction vector (3rd element is zero)
            Point transformDirection(const Point &dir) const
            {
                return Point(dir.x * _m[0] + dir.y * _m[1],
                               dir.x * _m[3] + dir.y * _m[4]);
            }
            
			// Transforms the given angle in radians
			float transformAngle(float angle) const;

			// Returns the vector that represents the direction of the transformed X-axis.
			Point x() const
			{
				return Point(_m[0], _m[3]);
			}

			// Returns the vector that represents the direction of the transformed Y-axis.
			Point y() const
			{
				return Point(_m[1], _m[4]);
			}

			// Returns the origin of the transformed coordinate system
			Point origin() const
			{
				return Point(_m[2], _m[5]);
			}

			// Returns the scaling along the transformed X-axis.
			float xScale() const
			{
				return x().mag();
			}

			// Returns the scaling along the transformed Y-axis.
			float yScale() const
			{
				return y().mag();
			}

			// Returns the clockwise angle from the transformed Y axis to the original Y axis.
			// This is not affected by horizontal reflection.
			float rotation() const;

			// Returns true if the coordinate system has been mirrored (i.e. is now left-handed).
			bool mirrored() const;

			const float *m() const { return _m; }

			static TransformMatrix translate(const Point &delta)
			{
				return TransformMatrix(1, 0, delta.x,
									   0, 1, delta.y);
			}

			static TransformMatrix rotate(float angle)
			{
				float c = cos(angle * DegreesToRadians);
				float s = sin(angle * DegreesToRadians);

				return TransformMatrix(c, -s, 0,
									   s, c, 0);
			}

			static TransformMatrix scale(float s)
			{
				return TransformMatrix(s, 0, 0,
									   0, s, 0);
			}

			// Returns a matrix to rotate <angle> degrees CCW around <center>.
			static TransformMatrix rotateAroundPoint(const Point &center, float angle);

			// Returns a matrix to reflect along the line parallel to the Y-axis containing <center>.
			static TransformMatrix mirrorAroundPoint(const Point &center);

			static const TransformMatrix identity;
			static const TransformMatrix mirrorX;

		protected:
			// Matrix values in row-major order.
			//
			// Indices:
			//   [0 1 2
			//    3 4 5]
			float _m[6];
	};
}
