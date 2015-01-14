#pragma once
#include <Geometry2d/Point.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <SystemState.hpp>
#include <QColor>
#include <QString>

namespace Planning
{
	class Path {
		public:
			virtual bool evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) const=0;
			virtual bool hit(const Geometry2d::CompositeShape &shape, unsigned int start = 0) const=0;
			virtual void draw(SystemState * const state, const QColor &color = Qt::black, const QString &layer = "Motion") const =0;
			virtual boost::optional<Geometry2d::Point> destination() const=0;
	};
}