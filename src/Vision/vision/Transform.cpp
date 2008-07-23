#include "Transform.h"
#include "Tracker.h"
#include "Distortion.h"
#include "../Config_File.h"

#include <QStringList>
#include <assert.h>
#include <math.h>
#include <Geometry/Segment.hpp>
#include <boost/foreach.hpp>

using namespace boost;
using namespace Geometry;
using namespace Eigen;

Vision::Transform::Transform()
{
	world_point[0] = Geometry::Point2d(0, 0);
    world_point[1] = Geometry::Point2d(1, 0);
    world_point[2] = Geometry::Point2d(0, 1);
    
	matrix.loadIdentity();
}

Geometry::Point2d Vision::Transform::transform(Geometry::Point2d in)
{
	Vector3f out = matrix * Vector3f(in.x, in.y, 1);
	return Geometry::Point2d(out[0], out[1]);
}

bool Vision::Transform::calculate_matrix(Distortion *distortion)
{
	// W = World coordinates
	// F = Frame coordinates
	// M = Transformation matrix
	//
	// W = M * F
	// M = W * F^-1
	Matrix3f w, f, f_inv;

	Geometry::Point2d image[3] =
	{
		distortion->undistort(image_point[0]),
		distortion->undistort(image_point[1]),
		distortion->undistort(image_point[2])
	};
	
	f[0] = image[0].x;
	f[1] = image[0].y;
	f[2] = 1;
	f[3] = image[1].x;
	f[4] = image[1].y;
	f[5] = 1;
	f[6] = image[2].x;
	f[7] = image[2].y;
	f[8] = 1;

	w[0] = world_point[0].x;
	w[1] = world_point[0].y;
	w[2] = 1;
	w[3] = world_point[1].x;
	w[4] = world_point[1].y;
	w[5] = 1;
	w[6] = world_point[2].x;
	w[7] = world_point[2].y;
	w[8] = 1;
	
	bool invertible = false;
	f.computeInverseSafely(&f_inv, &invertible);
	
	if (invertible)
	{
		matrix = w * f_inv;
	}
	
	return invertible;
}

void Vision::Transform::load(QDomElement element)
{
	QString text = element.text();
	QStringList tokens = text.split(QRegExp("\\s"), QString::SkipEmptyParts);
	if (tokens.size() != 9)
	{
		printf("Vision::Transform::load: Wrong number of matrix elements: have %d, need 9\n", tokens.size());
	} else {
		for (int i = 0; i < 9; ++i)
		{
			bool ok = false;
			matrix[i] = tokens[i].toFloat(&ok);
			if (!ok)
			{
				printf("Vision::Transform::load: Bad matrix value \"%s\"\n", tokens[i].toAscii().constData());
			}
		}
	}
}

void Vision::Transform::save(QDomElement element)
{
	QString text = "\n";
	for (int i = 0; i < 9; ++i)
	{
		text += QString::number(matrix[i]);
		
		if ((i % 3) == 2)
		{
			text += '\n';
		} else {
			text += ' ';
		}
	}
	element.appendChild(element.ownerDocument().createTextNode(text));
}
