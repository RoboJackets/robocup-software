#include "Patch.hpp"
#include "Primitives.hpp"
#include "RenderUtils.hpp"
#include <Constants.hpp>

namespace rendering {

/// Primitive

void Primitive::translate(const QVector3D &t)
{
	const QVector3D st = t * _scale;
	for (int i = 0; i < parts.count(); ++i)
		parts[i]->translate(st);
}
void Primitive::rotate(qreal deg, QVector3D axis)
{
	for (int i = 0; i < parts.count(); ++i)
		parts[i]->rotate(deg, axis);
}

void Primitive::setColor(QColor c)
{
	for (int i = 0; i < parts.count(); ++i)
		qSetColor(parts[i]->faceColor, c);
}

/// RectPrism

RectPrism::RectPrism(Geometry *g, qreal scale, qreal widthUn, qreal heightUn, qreal depthUn)
: Primitive(scale)
{
	// scale measurements
	qreal width  = widthUn  * _scale;
	qreal height = heightUn * _scale;
	qreal depth  = depthUn  * _scale;

	enum { bl, br, tr, tl };
	Patch *fb = new Patch(g);
	fb->setSmoothing(Patch::Faceted);

	// front face
	QVector<QVector3D> r(4);
	r[br].setX(width);
	r[tr].setX(width);
	r[tr].setY(height);
	r[tl].setY(height);
	QVector3D adjToCenter(-width / 2.0, -height / 2.0, depth / 2.0);
	for (int i = 0; i < 4; ++i)
		r[i] += adjToCenter;
	fb->addQuad(r[bl], r[br], r[tr], r[tl]);

	// back face
	QVector<QVector3D> s = extrude(r, depth);
	fb->addQuad(s[tl], s[tr], s[br], s[bl]);

	// side faces
	Patch *sides = new Patch(g);
	sides->setSmoothing(Patch::Faceted);
	sides->addQuad(s[bl], s[br], r[br], r[bl]);
	sides->addQuad(s[br], s[tr], r[tr], r[br]);
	sides->addQuad(s[tr], s[tl], r[tl], r[tr]);
	sides->addQuad(s[tl], s[bl], r[bl], r[tl]);

	parts << fb << sides;
}

/// RectTorus

RectTorus::RectTorus(Geometry *g, qreal scale, qreal iRad, qreal oRad, qreal depth, int k)
: Primitive(scale)
{
	QVector<QVector3D> inside;
	QVector<QVector3D> outside;
	for (int i = 0; i < k; ++i) {
		qreal angle = (i * 2 * M_PI) / k;
		inside << QVector3D(iRad * qSin(angle), iRad * qCos(angle), depth / 2.0);
		outside << QVector3D(oRad * qSin(angle), oRad * qCos(angle), depth / 2.0);
	}
	inside << QVector3D(0.0, iRad, 0.0);
	outside << QVector3D(0.0, oRad, 0.0);
	QVector<QVector3D> in_back = extrude(inside, depth);
	QVector<QVector3D> out_back = extrude(outside, depth);

	// Create front, back and sides as separate patches so that smooth normals
	// are generated for the curving sides, but a faceted edge is created between
	// sides and front/back
	Patch *front = new Patch(g);
	for (int i = 0; i < k; ++i)
		front->addQuad(outside[i], inside[i],
				inside[(i + 1) % k], outside[(i + 1) % k]);
	Patch *back = new Patch(g);
	for (int i = 0; i < k; ++i)
		back->addQuad(in_back[i], out_back[i],
				out_back[(i + 1) % k], in_back[(i + 1) % k]);
	Patch *is = new Patch(g);
	for (int i = 0; i < k; ++i)
		is->addQuad(in_back[i], in_back[(i + 1) % k],
				inside[(i + 1) % k], inside[i]);
	Patch *os = new Patch(g);
	for (int i = 0; i < k; ++i)
		os->addQuad(out_back[(i + 1) % k], out_back[i],
				outside[i], outside[(i + 1) % k]);
	parts << front << back << is << os;
}

/// Simple Cylinder

Cylinder::Cylinder(Geometry *g, qreal scale, qreal height_, qreal radius_, int k)
: Primitive(scale)
{
	// scaled measurements
	const qreal radius = radius_ * _scale;
	const qreal height = height_ * _scale;

	// Create rings for cylinder - reverse ordering on top and bottom to flip normals
	const qreal offset = 0.0; // used for debugging structure
	QVector<QVector3D> ring_top, ring_bottom;
	qreal angle_increment = (2 * M_PI) / k;
	for (int i = 0; i <= k; ++i) {
		qreal angle = i * angle_increment;
		qreal x = radius * qSin(angle);
		qreal y = radius * qCos(angle);
		ring_top.push_front(QVector3D(x, y, height + offset));
		ring_bottom.push_back(QVector3D(x, y, offset));
	}

	// create 3 patches, with a single patch for the side
	Patch *top = new Patch(g);
	top->addPolygon(ring_top);
	Patch *bottom = new Patch(g);
	bottom->addPolygon(ring_bottom);

	// wrap around outside
	QVector3D vert_edge(0.0, 0.0, -height);
	Patch *side = new Patch(g);
	side->sm = Patch::Smooth;
	for (int i=0; i<k; ++i)
		side->addQuad(
				ring_top[i], // top left
				ring_top[i] + vert_edge, // bottom left
				ring_top[i+1] + vert_edge, // bottom right
				ring_top[i+1]); // top right

	parts << top << bottom << side;
}

/// Shape for a robot

SSLRobotShape::SSLRobotShape(Geometry *g, qreal scale, int k)
: Primitive(scale)
{
	// scaled measurements
	const qreal radius = Robot_Radius * _scale;
	const qreal height = Robot_Height * _scale;
	const qreal flat_front_angle = M_PI_4; // assume x is forward on robot, rotation ccw to corner

	const qreal offset = 0.2;
	// Create a simple cylinder for now
	QVector<QVector3D> ring_top, ring_bottom;
	qreal angle_increment = (2 * M_PI - 2 * flat_front_angle) / k;
	for (int i = 0; i <= k; ++i) {
		qreal angle = flat_front_angle + i * angle_increment;
		qreal x = radius * qSin(angle);
		qreal y = radius * qCos(angle);
		ring_top.push_front(QVector3D(x, y, height + offset));
		ring_bottom.push_back(QVector3D(x, y, offset));
	}

	// create 4 patches, with a single patch for the side
	Patch *top = new Patch(g);
	top->addPolygon(ring_top);
	Patch *bottom = new Patch(g);
	bottom->addPolygon(ring_bottom);

	// wrap around outside
	QVector3D vert_edge(0.0, 0.0, -height);
	Patch *side = new Patch(g);
	side->sm = Patch::Smooth;
	for (int i=0; i<k; ++i)
		side->addQuad(
				ring_top[i], // top left
				ring_top[i] + vert_edge, // bottom left
				ring_top[i+1] + vert_edge, // bottom right
				ring_top[i+1]); // top right

	Patch *front = new Patch(g);
	front->addQuad(
				ring_top[k],
				ring_bottom[0], // bottom left (reversed order)
				ring_bottom[k],
				ring_top[0]);

	parts << top << bottom << side << front;
}

} // \namespace rendering
