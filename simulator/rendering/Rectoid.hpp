#pragma once

#include <QMatrix4x4>
#include <QVector3D>

namespace rendering {

class Patch;
struct Geometry;

class Rectoid
{
protected:
	qreal _scale;

public:

	Rectoid(qreal scale) : _scale(scale) {}

	/** translate in 3D frame - z is up */
	void translate(const QVector3D &t);

	/** axis-angle rotations */
	void rotate(qreal deg, QVector3D axis);

	// No special Rectoid destructor - the parts are fetched out of this member
	// variable, and destroyed by the new owner
	QList<Patch*> parts;
};

// Subclasses for variations
class RectPrism : public Rectoid
{
public:
	/** centered at origin */
	RectPrism(Geometry *g, qreal scale, qreal width, qreal height, qreal depth);
};

class RectTorus : public Rectoid
{
public:
	RectTorus(Geometry *g, qreal scale, qreal iRad, qreal oRad, qreal depth, int numSectors);
};


} // \namespace rendering
