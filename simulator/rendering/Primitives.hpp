#pragma once

#include <QMatrix4x4>
#include <QVector3D>

namespace rendering {

class Patch;
struct Geometry;

class Primitive
{
protected:
	qreal _scale;

public:

	Primitive(qreal scale) : _scale(scale) {}

	/** translate in 3D frame - z is up */
	void translate(const QVector3D &t);

	/** axis-angle rotations */
	void rotate(qreal deg, QVector3D axis);

	/** sets the color for all parts in this primitive */
	void setColor(QColor c);


	// No special Primitive destructor - the parts are fetched out of this member
	// variable, and destroyed by the new owner
	QList<Patch*> parts;
};

// Subclasses for variations
class RectPrism : public Primitive
{
public:
	/** centered at origin */
	RectPrism(Geometry *g, qreal scale, qreal width, qreal height, qreal depth);
};

class RectTorus : public Primitive
{
public:
	RectTorus(Geometry *g, qreal scale, qreal iRad, qreal oRad, qreal depth, int numSectors);
};

class Cylinder : public Primitive
{
public:
	Cylinder(Geometry *g, qreal scale, qreal height, qreal radius, int numSectors);
};

class SSLRobotShape : public Primitive
{
public:
	SSLRobotShape(Geometry *g, qreal scale, int numSectors);
};

class Sphere : public Primitive
{
public:
	Sphere(Geometry *g, qreal scale, qreal radius, int numSectors, int numRows);
};

} // \namespace rendering
