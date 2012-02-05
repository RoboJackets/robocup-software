#pragma once

#include <QMatrix4x4>
#include <QVector3D>

namespace rendering {

class Patch;
struct Geometry;

class Rectoid
{
public:
	void translate(const QVector3D &t);
	void rotate(qreal deg, QVector3D axis);

	// No special Rectoid destructor - the parts are fetched out of this member
	// variable, and destroyed by the new owner
	QList<Patch*> parts;
};

class RectPrism : public Rectoid
{
public:
	RectPrism(Geometry *g, qreal width, qreal height, qreal depth);
};

class RectTorus : public Rectoid
{
public:
	RectTorus(Geometry *g, qreal iRad, qreal oRad, qreal depth, int numSectors);
};


} // \namespace rendering
