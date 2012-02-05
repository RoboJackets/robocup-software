#pragma once

#include <QGLWidget>
#include <QMatrix4x4>
#include <QVector3D>

#include <qmath.h>

namespace rendering {

struct Geometry;

class Patch
{
public:
	enum Smoothing { Faceted, Smooth };
	Patch(Geometry *);
	void setSmoothing(Smoothing s) { sm = s; }
	void translate(const QVector3D &t);
	void rotate(qreal deg, QVector3D axis);
	void draw() const;
	void addTri(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &n);
	void addQuad(const QVector3D &a, const QVector3D &b,  const QVector3D &c, const QVector3D &d);

	GLushort start;
	GLushort count;
	GLushort initv;

	GLfloat faceColor[4];
	QMatrix4x4 mat;
	Smoothing sm;
	Geometry *geom;
};

} // \namespace rendering
