#include "TransformMatrix.hpp"
#include "util.h"

using namespace Geometry;

const TransformMatrix TransformMatrix::identity(1, 0, 0,
		0, 1, 0);

const TransformMatrix TransformMatrix::mirrorX(-1, 0, 0,
		0, 1, 0);

TransformMatrix::TransformMatrix(const Geometry::Point2d &origin, float rotation, bool mirror, float s)
{
	// Set up translation
	_m[0] = 1;
	_m[1] = 0;
	_m[2] = origin.x;
	_m[3] = 0;
	_m[4] = 1;
	_m[5] = origin.y;

	*this *= rotate(rotation);
	*this *= scale(s);
	if (mirror)
		*this *= mirrorX;
}

float TransformMatrix::transformAngle(float angle) const
{
	// Multiply the matrix by a unit vector in the given direction
	// with a 3rd element of zero and find the direction of the result.
	
	float px = cos(angle);
	float py = sin(angle);
	
	float rx = px * _m[0] + py * _m[1];
	float ry = px * _m[3] + py * _m[4];
	
	return atan2(ry, rx);
}

TransformMatrix TransformMatrix::rotateAroundPoint2d(const Point2d &center, float angle)
{
	TransformMatrix xf = translate(center);
	xf *= rotate(angle);
	xf *= translate(-center);

	return xf;
}

TransformMatrix TransformMatrix::mirrorAroundPoint2d(const Point2d &center)
{
	TransformMatrix xf = translate(center);
	xf *= mirrorX;
	xf *= translate(-center);

	return xf;
}

float TransformMatrix::rotation() const
{
	float angle = atan2(_m[4], _m[1]) / DEG_TO_RAD - 90;
	if (angle < 0)
		angle += 360;

	return angle;
}

bool TransformMatrix::mirrored() const
{
	// This matrix contains a reflection iff the Z component of
	// the transformed x vector cross the transformed y vector
	// is negative.  Conveniently, this is the determinant of the
	// matrix formed by the two vectors.
	//
	// In a right-handed coordinate system, +X cross +Y = +Z.
	// A reflection flips the coordinate system to be left-handed
	// so the (right-handed) cross product becomes -Z.
	return (_m[0] * _m[4] - _m[1] * _m[3]) < 0;
}
