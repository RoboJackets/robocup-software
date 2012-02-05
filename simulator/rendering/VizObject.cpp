#include "VizObject.hpp"
#include "Geometry.hpp"
#include "Rectoid.hpp"
#include "Patch.hpp"
#include "RenderUtils.hpp"

#include <Constants.hpp>

using namespace rendering;

/// VizObject Implementation
VizObject::VizObject(QObject *parent)
: QObject(parent)
, geom(new Geometry())
{
}

VizObject::~VizObject()
{
	qDeleteAll(parts);
	delete geom;
}

void VizObject::setColor(QColor c)
{
	for (int i = 0; i < parts.count(); ++i)
		qSetColor(parts[i]->faceColor, c);
}

void VizObject::draw() const
{
	geom->loadArrays();

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	for (int i = 0; i < parts.count(); ++i)
		parts[i]->draw();

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

///  Robot visualization
RobotBody::RobotBody(QObject *parent)
: VizObject(parent)
{
	buildGeometry();
}

void RobotBody::buildGeometry()
{
	// TODO: fill in
}


///  Field visualization - all static objects
RCField::RCField(QObject *parent)
: VizObject(parent)
{
	buildGeometry();
}

void RCField::buildGeometry()
{
	static const qreal field_length = Field_Length + 2.f * Field_Border;
	static const qreal field_width = Field_Width + 2.f * Field_Border;
	static const qreal field_depth = 0.1;
	static const qreal wall_height = 0.3;
	static const qreal wall_thickness = 0.05;
	static const qreal scale = 0.1;

	// add main plane
	RectPrism base(geom, scale, field_length, field_width, field_depth);
	base.setColor(Qt::green);

	// set up transforms
	QVector3D onfield(0.f, 0.f, (0.5 * wall_height - 0.5 * field_depth));

	// add lengthwise walls
	RectPrism longwall1(geom, scale, field_length, wall_thickness, wall_height);
	longwall1.translate(onfield);
	longwall1.translate(QVector3D(0.f, -0.5 * (field_width + wall_thickness), 0.f));
	longwall1.setColor(Qt::white);

	RectPrism longwall2(geom, scale, field_length, wall_thickness, wall_height);
	longwall2.translate(onfield);
	longwall2.translate(QVector3D(0.f, 0.5 * (field_width + wall_thickness), 0.f));
	longwall2.setColor(Qt::white);

	// add end-walls - cap corners
	RectPrism shortwall1(geom, scale, wall_thickness, field_width + 2.f * wall_thickness, wall_height);
	shortwall1.translate(onfield);
	shortwall1.translate(QVector3D(-0.5 * (field_length + wall_thickness), 0.f, 0.f));
	shortwall1.setColor(Qt::white);

	RectPrism shortwall2(geom, scale, wall_thickness, field_width + 2.f * wall_thickness, wall_height);
	shortwall2.translate(onfield);
	shortwall2.translate(QVector3D( 0.5 * (field_length + wall_thickness), 0.f, 0.f));
	shortwall2.setColor(Qt::white);

	// assemble
	parts << base.parts << longwall1.parts << longwall2.parts << shortwall1.parts << shortwall2.parts;
	geom->finalize();
}

///  Qt Logo Implementation

QtLogo::QtLogo(QObject *parent, int divisions, qreal scale)
: VizObject(parent), _divisions(divisions), _scale(scale)
{
	buildGeometry();
}

void QtLogo::buildGeometry()
{
	static const qreal tee_height = 0.311126;
	static const qreal cross_width = 0.25;
	static const qreal bar_thickness = 0.113137;
	static const qreal logo_depth = 0.10;

	qreal cw = cross_width;
	qreal bt = bar_thickness;
	qreal ld = logo_depth;
	qreal th = tee_height;

	RectPrism cross(geom, _scale, cw, bt, ld);
	RectPrism stem(geom, _scale, bt, th, ld);

	QVector3D z(0.0, 0.0, 1.0);
	cross.rotate(45.0, z);
	stem.rotate(45.0, z);

	qreal stem_downshift = (th + bt) / 2.0;
	stem.translate(QVector3D(0.0, -stem_downshift, 0.0));

	RectTorus body(geom, _scale, 0.20, 0.30, 0.1, _divisions);

	parts << stem.parts << cross.parts << body.parts;

	geom->finalize();
}

