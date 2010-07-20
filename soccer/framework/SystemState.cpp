#include <framework/SystemState.hpp>
#include <protobuf/LogFrame.pb.h>
#include <LogUtils.hpp>

#include <stdio.h>

using namespace Packet;

int SystemState::findDebugLayer(QString layer)
{
	if (layer.isNull())
	{
		layer = "Debug";
	}
	
	DebugLayerMap::const_iterator i = _debugLayerMap.find(layer);
	if (i == _debugLayerMap.end())
	{
		// New layer
		int n = _numDebugLayers++;
		_debugLayerMap[layer] = n;
		_debugLayers.append(layer);
		return n;
	} else {
		// Existing layer
		return i.value();
	}
}

void SystemState::drawPath(const Geometry2d::Point* pts, int n, const QColor& qc, const QString& layer)
{
	DebugPath *dbg = logFrame->add_debug_paths();
	dbg->set_layer(findDebugLayer(layer));
	for (int i = 0; i < n; ++i)
	{
		pts[i].set(dbg->add_points());
	}
	dbg->set_color(color(qc));
}

void SystemState::drawPolygon(const Geometry2d::Point* pts, int n, const QColor& qc, const QString &layer)
{
	DebugPath *dbg = logFrame->add_debug_polygons();
	dbg->set_layer(findDebugLayer(layer));
	for (int i = 0; i < n; ++i)
	{
		pts[i].set(dbg->add_points());
	}
	dbg->set_color(color(qc));
}

void SystemState::drawCircle(const Geometry2d::Point& center, float radius, const QColor& qc, const QString &layer)
{
	DebugCircle *dbg = logFrame->add_debug_circles();
	dbg->set_layer(findDebugLayer(layer));
	center.set(dbg->mutable_center());
	dbg->set_radius(radius);
	dbg->set_color(color(qc));
}

void SystemState::drawLine(const Geometry2d::Line& line, const QColor& qc, const QString &layer)
{
	DebugPath *dbg = logFrame->add_debug_paths();
	dbg->set_layer(findDebugLayer(layer));
	line.pt[0].set(dbg->add_points());
	line.pt[1].set(dbg->add_points());
	dbg->set_color(color(qc));
}

void SystemState::drawText(const QString& text, const Geometry2d::Point& pos, const QColor& qc, const QString &layer)
{
	DebugText *dbg = logFrame->add_debug_texts();
	dbg->set_layer(findDebugLayer(layer));
	dbg->set_text(text.toStdString());
	pos.set(dbg->mutable_pos());
	dbg->set_color(color(qc));
}
