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

///  Field visualization - all static objects
RCField::RCField(QObject *parent)
: VizObject(parent)
{
	buildGeometry();
}

void RCField::buildGeometry()
{
	static const qreal field_len = Field_Length + 2.f * Field_Border;
	static const qreal field_width = Field_Width + 2.f * Field_Border;
	static const qreal field_depth = 0.1;

	// add main plane
	RectPrism base(geom, field_len, field_width, field_depth);

	// add lengthwise walls

	// assemble
	parts << base.parts;
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

	qreal cw = cross_width   * _scale;
	qreal bt = bar_thickness * _scale;
	qreal ld = logo_depth    * _scale;
	qreal th = tee_height    *_scale;

	RectPrism cross(geom, cw, bt, ld);
	RectPrism stem(geom, bt, th, ld);

	QVector3D z(0.0, 0.0, 1.0);
	cross.rotate(45.0, z);
	stem.rotate(45.0, z);

	qreal stem_downshift = (th + bt) / 2.0;
	stem.translate(QVector3D(0.0, -stem_downshift, 0.0));

	RectTorus body(geom, 0.20, 0.30, 0.1, _divisions);

	parts << stem.parts << cross.parts << body.parts;

	geom->finalize();
}

