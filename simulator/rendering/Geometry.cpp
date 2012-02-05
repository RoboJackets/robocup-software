#include <QGLWidget>
#include <QMatrix4x4>
#include <QVector3D>

#include <qmath.h>

#include "Geometry.hpp"

namespace rendering {

void Geometry::loadArrays() const
{
	glVertexPointer(3, GL_FLOAT, 0, vertices.constData());
	glNormalPointer(GL_FLOAT, 0, normals.constData());
}

void Geometry::finalize()
{
	// TODO: add vertex buffer uploading here

	// Finish smoothing normals by ensuring accumulated normals are returned
	// to length 1.0.
	for (int i = 0; i < normals.count(); ++i)
		normals[i].normalize();
}

void Geometry::appendSmooth(const QVector3D &a, const QVector3D &n, int from)
{
	// Smooth normals are acheived by averaging the normals for faces meeting
	// at a point.  First find the point in geometry already generated
	// (working backwards, since most often the points shared are between faces
	// recently added).
	int v = vertices.count() - 1;
	for ( ; v >= from; --v)
		if (qFuzzyCompare(vertices[v], a))
			break;
	if (v < from)
	{
		// The vert was not found so add it as a new one, and initialize
		// its corresponding normal
		v = vertices.count();
		vertices.append(a);
		normals.append(n);
	}
	else
	{
		// Vert found, accumulate normals into corresponding normal slot.
		// Must call finalize once finished accumulating normals
		normals[v] += n;
	}
	// In both cases (found or not) reference the vert via its index
	faces.append(v);
}

void Geometry::appendFaceted(const QVector3D &a, const QVector3D &n)
{
	// Faceted normals are achieved by duplicating the vert for every
	// normal, so that faces meeting at a vert get a sharp edge.
	int v = vertices.count();
	vertices.append(a);
	normals.append(n);
	faces.append(v);
}

} // \namespace rendering
