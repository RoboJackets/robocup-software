#include "DebugDrawer.hpp"
#include "LogUtils.hpp"

int DebugDrawer::findDebugLayer(QString layer) {
    if (layer.isNull()) {
        layer = "Debug";
    }

    QMap<QString, int>::const_iterator i = _debugLayerMap.find(layer);
    if (i == _debugLayerMap.end()) {
        // New layer
        int n = _numDebugLayers++;
        _debugLayerMap[layer] = n;
        _debugLayers.append(layer);
        return n;
    }
    // Existing layer
    return i.value();
}

void DebugDrawer::drawPolygon(const geometry2d::Point* pts, int n,
                              const QColor& qc, const QString& layer) {
    Packet::DebugPath* dbg = _logFrame.add_debug_polygons();
    dbg->set_layer(findDebugLayer(layer));
    for (int i = 0; i < n; ++i) {
        *dbg->add_points() = pts[i];
    }
    dbg->set_color(color(qc));
}

void DebugDrawer::drawPolygon(const std::vector<geometry2d::Point>& pts,
                              const QColor& qc, const QString& layer) {
    drawPolygon(pts.data(), pts.size(), qc, layer);
}

void DebugDrawer::drawPolygon(const geometry2d::Polygon& polygon,
                              const QColor& qc, const QString& layer) {
    this->drawPolygon(polygon.vertices, qc, layer);
}

void DebugDrawer::drawCircle(geometry2d::Point center, float radius,
                             const QColor& qc, const QString& layer) {
    Packet::DebugCircle* dbg = _logFrame.add_debug_circles();
    dbg->set_layer(findDebugLayer(layer));
    *dbg->mutable_center() = center;
    dbg->set_radius(radius);
    dbg->set_color(color(qc));
}

void DebugDrawer::drawArc(const geometry2d::Arc& arc, const QColor& qc,
                          const QString& layer) {
    Packet::DebugArc* dbg = _logFrame.add_debug_arcs();
    dbg->set_layer(findDebugLayer(layer));
    *dbg->mutable_center() = arc.center();
    dbg->set_radius(arc.radius());
    dbg->set_start(arc.start());
    dbg->set_end(arc.end());
    dbg->set_color(color(qc));
}

void DebugDrawer::drawShape(const std::shared_ptr<geometry2d::Shape>& obs,
                            const QColor& color, const QString& layer) {
    std::shared_ptr<geometry2d::Circle> circObs =
        std::dynamic_pointer_cast<geometry2d::Circle>(obs);
    std::shared_ptr<geometry2d::Polygon> polyObs =
        std::dynamic_pointer_cast<geometry2d::Polygon>(obs);
    std::shared_ptr<geometry2d::CompositeShape> compObs =
        std::dynamic_pointer_cast<geometry2d::CompositeShape>(obs);
    if (circObs)
        drawCircle(circObs->center, circObs->radius(), color, layer);
    else if (polyObs)
        drawPolygon(polyObs->vertices, color, layer);
    else if (compObs) {
        for (const std::shared_ptr<geometry2d::Shape>& obs :
             compObs->subshapes())
            drawShape(obs, color, layer);
    }
}

void DebugDrawer::drawShapeSet(const geometry2d::ShapeSet& shapes,
                               const QColor& color, const QString& layer) {
    for (auto& shape : shapes.shapes()) {
        drawShape(shape, color, layer);
    }
}

void DebugDrawer::drawLine(const geometry2d::Segment& line, const QColor& qc,
                           const QString& layer) {
    Packet::DebugPath* dbg = _logFrame.add_debug_paths();
    dbg->set_layer(findDebugLayer(layer));
    *dbg->add_points() = line.pt[0];
    *dbg->add_points() = line.pt[1];
    dbg->set_color(color(qc));
}

void DebugDrawer::drawLine(geometry2d::Point p0, geometry2d::Point p1,
                           const QColor& color, const QString& layer) {
    drawLine(geometry2d::Segment(p0, p1), color, layer);
}

void DebugDrawer::drawText(const QString& text, geometry2d::Point pos,
                           const QColor& qc, const QString& layer) {
    Packet::DebugText* dbg = _logFrame.add_debug_texts();
    dbg->set_layer(findDebugLayer(layer));
    dbg->set_text(text.toStdString());
    *dbg->mutable_pos() = pos;
    dbg->set_color(color(qc));
}

void DebugDrawer::drawSegment(const geometry2d::Segment& line, const QColor& qc,
                              const QString& layer) {
    Packet::DebugPath* dbg = _logFrame.add_debug_paths();
    dbg->set_layer(findDebugLayer(layer));
    *dbg->add_points() = line.pt[0];
    *dbg->add_points() = line.pt[1];
    dbg->set_color(color(qc));
}
