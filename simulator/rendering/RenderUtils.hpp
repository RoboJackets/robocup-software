#pragma once

#include <QGLWidget>
#include <QMatrix4x4>
#include <QVector3D>

namespace rendering {

static inline void qSetColor(float colorVec[], QColor c)
{
	colorVec[0] = c.redF();
	colorVec[1] = c.greenF();
	colorVec[2] = c.blueF();
	colorVec[3] = c.alphaF();
}

static inline void qMultMatrix(const QMatrix4x4 &mat)
{
	if (sizeof(qreal) == sizeof(GLfloat))
		glMultMatrixf((GLfloat*)mat.constData());
#ifndef QT_OPENGL_ES
	else if (sizeof(qreal) == sizeof(GLdouble))
		glMultMatrixd((GLdouble*)mat.constData());
#endif
	else
	{
		GLfloat fmat[16];
		qreal const *r = mat.constData();
		for (int i = 0; i < 16; ++i)
			fmat[i] = r[i];
		glMultMatrixf(fmat);
	}
}

static inline QVector<QVector3D> extrude(const QVector<QVector3D> &verts, qreal depth)
		{
	QVector<QVector3D> extr = verts;
	for (int v = 0; v < extr.count(); ++v)
		extr[v].setZ(extr[v].z() - depth);
	return extr;
		}

} // \namespace rendering
