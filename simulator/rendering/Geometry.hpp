#pragma once

#include <QGLWidget>
#include <QMatrix4x4>
#include <QVector3D>

#include <qmath.h>

namespace rendering {

struct Geometry
{
	QVector<GLushort> faces;
	QVector<QVector3D> vertices;
	QVector<QVector3D> normals;
	void appendSmooth(const QVector3D &a, const QVector3D &n, int from);
	void appendFaceted(const QVector3D &a, const QVector3D &n);
	void finalize();
	void loadArrays() const;
};

} // \namespace rendering
