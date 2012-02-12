#pragma once

#include <QObject>
#include <QColor>
#include <QVector3D>

namespace rendering {

class Patch;
struct Geometry;

/**
 * Object that can be visualized - use this as a base class
 */
class VizObject : public QObject
{
public:
	VizObject(QObject *parent, qreal scale = 0.1);
	virtual ~VizObject();

	/** sets color for all parts */
	virtual void setColor(QColor c);
	virtual void draw() const;

	/** move object */
	virtual void translate(const QVector3D& pos);

	virtual void rotate(qreal deg, const QVector3D& axis);

	/** build the specific structure */
	virtual void buildGeometry() = 0;

protected:
	QList<Patch *> parts;
	Geometry *geom;
	qreal _scale;
};

/**
 * Robot body class - solid colors
 */
class RobotBody : public VizObject
{
public:
	RobotBody(QObject *parent, qreal scale);
	virtual ~RobotBody() {}

	void buildGeometry();
};

/**
 * Ball object - a sphere that can rotate
 */
class Ball : public VizObject
{
public:
	Ball(QObject *parent, qreal scale);
	virtual ~Ball() {}

	void buildGeometry();
};

/**
 * Field visualization - all static objects
 */
class RCField : public VizObject
{
public:
	RCField(QObject *parent, qreal scale);
	virtual ~RCField() {}

	void buildGeometry();
};

///**
// * Qt Logo example visualization
// */
//class QtLogo : public VizObject
//{
//protected:
//	int _divisions;
//	qreal _scale;
//
//public:
//	QtLogo(QObject *parent, int d = 64, qreal s = 1.0);
//	virtual ~QtLogo() {}
//
//	void buildGeometry();
//};

} // \namespace rendering
