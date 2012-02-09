#pragma once

#include <QObject>
#include <QColor>

namespace rendering {

class Patch;
struct Geometry;

/**
 * Object that can be visualized - use this as a base class
 */
class VizObject : public QObject
{
public:
	VizObject(QObject *parent);
	virtual ~VizObject();

	/** sets color for all parts */
	virtual void setColor(QColor c);
	virtual void draw() const;

	/** build the specific structure */
	virtual void buildGeometry() = 0;

protected:
	QList<Patch *> parts;
	Geometry *geom;
};

/**
 * Robot body class - solid colors
 */
class RobotBody : public VizObject
{
public:
	RobotBody(QObject *parent);
	virtual ~RobotBody() {}

	void buildGeometry();
};

/**
 * Ball object - a sphere that can rotate
 */
class Ball : public VizObject
{
public:
	Ball(QObject *parent);
	virtual ~Ball() {}

	void buildGeometry();
};

/**
 * Field visualization - all static objects
 */
class RCField : public VizObject
{
public:
	RCField(QObject *parent);
	virtual ~RCField() {}

	void buildGeometry();
};

/**
 * Qt Logo example visualization
 */
class QtLogo : public VizObject
{
protected:
	int _divisions;
	qreal _scale;

public:
	QtLogo(QObject *parent, int d = 64, qreal s = 1.0);
	virtual ~QtLogo() {}

	void buildGeometry();
};

} // \namespace rendering
