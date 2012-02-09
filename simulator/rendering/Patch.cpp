#include <QGLWidget>
#include <QMatrix4x4>
#include <QVector3D>

#include <qmath.h>

#include "Patch.hpp"
#include "Geometry.hpp"
#include "RenderUtils.hpp"

namespace rendering {

Patch::Patch(Geometry *g)
: start(g->faces.count())
, count(0)
, initv(g->vertices.count())
, sm(Patch::Smooth)
, geom(g)
{
	qSetColor(faceColor, QColor(Qt::darkGray));
}

void Patch::rotate(qreal deg, QVector3D axis)
{
	mat.rotate(deg, axis);
}

void Patch::translate(const QVector3D &t)
{
	mat.translate(t);
}

void Patch::draw() const
{
	glPushMatrix();
	qMultMatrix(mat);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, faceColor);

	const GLushort *indices = geom->faces.constData();
	glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_SHORT, indices + start);
	glPopMatrix();
}

void Patch::addTri(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &n)
{
	QVector3D norm = n.isNull() ? QVector3D::normal(a, b, c) : n;
	if (sm == Smooth)
	{
		geom->appendSmooth(a, norm, initv);
		geom->appendSmooth(b, norm, initv);
		geom->appendSmooth(c, norm, initv);
	}
	else
	{
		geom->appendFaceted(a, norm);
		geom->appendFaceted(b, norm);
		geom->appendFaceted(c, norm);
	}
	count += 3;
}

void Patch::addQuad(const QVector3D &a, const QVector3D &b,  const QVector3D &c, const QVector3D &d)
{
	QVector3D norm = QVector3D::normal(a, b, c);
	if (sm == Smooth)
	{
		addTri(a, b, c, norm);
		addTri(a, c, d, norm);
	}
	else
	{
		// If faceted share the two common verts
		addTri(a, b, c, norm);
		int k = geom->vertices.count();
		geom->appendSmooth(a, norm, k);
		geom->appendSmooth(c, norm, k);
		geom->appendFaceted(d, norm);
		count += 3;
	}
}

// Assumes all in the same plane
void Patch::addPolygon(const QVector<QVector3D>& vertices) {
	const int n = vertices.size();
	// pull out first 3 vertices to get a normal
	if (n < 3) return;
	const QVector3D& a(vertices[0]), b(vertices[1]), c(vertices[2]);
	QVector3D norm = QVector3D::normal(a, b, c);

	// only support smooth operations for polygons
	Smoothing old = sm;
	sm = Smooth;
	for (int i=1; i<n-1; ++i)
		addTri(vertices[0], vertices[i], vertices[i+1], norm);
	sm = old;
}

} // \namespace rendering
