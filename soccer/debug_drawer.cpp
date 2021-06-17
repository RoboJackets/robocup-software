#include "debug_drawer.hpp"

#include "log_utils.hpp"

int DebugDrawer::find_debug_layer(QString layer) {
    if (layer.isNull()) {
        layer = "Debug";
    }

    QMap<QString, int>::const_iterator i = debug_layer_map_.find(layer);
    if (i == debug_layer_map_.end()) {
        // New layer
        int n = num_debug_layers_++;
        debug_layer_map_[layer] = n;
        debug_layers_.append(layer);
        return n;
    }
    // Existing layer
    return i.value();
}

void DebugDrawer::draw_polygon(const rj_geometry::Point* pts, int n, const QColor& qc,
                               const QString& layer) {
    Packet::DebugPath* dbg = log_frame_.add_debug_polygons();
    dbg->set_layer(find_debug_layer(layer));
    for (int i = 0; i < n; ++i) {
        *dbg->add_points() = pts[i];
    }
    dbg->set_color(color(qc));
}

void DebugDrawer::draw_polygon(const std::vector<rj_geometry::Point>& pts, const QColor& qc,
                               const QString& layer) {
    draw_polygon(pts.data(), pts.size(), qc, layer);
}

void DebugDrawer::draw_polygon(const rj_geometry::Polygon& polygon, const QColor& qc,
                               const QString& layer) {
    this->draw_polygon(polygon.vertices, qc, layer);
}

void DebugDrawer::draw_circle(rj_geometry::Point center, float radius, const QColor& qc,
                              const QString& layer) {
    Packet::DebugCircle* dbg = log_frame_.add_debug_circles();
    dbg->set_layer(find_debug_layer(layer));
    *dbg->mutable_center() = center;
    dbg->set_radius(radius);
    dbg->set_color(color(qc));
}

void DebugDrawer::draw_arc(const rj_geometry::Arc& arc, const QColor& qc, const QString& layer) {
    Packet::DebugArc* dbg = log_frame_.add_debug_arcs();
    dbg->set_layer(find_debug_layer(layer));
    *dbg->mutable_center() = arc.center();
    dbg->set_radius(arc.radius());
    dbg->set_start(arc.start());
    dbg->set_end(arc.end());
    dbg->set_color(color(qc));
}

void DebugDrawer::draw_shape(const std::shared_ptr<rj_geometry::Shape>& obs, const QColor& color,
                             const QString& layer) {
    std::shared_ptr<rj_geometry::Circle> circ_obs =
        std::dynamic_pointer_cast<rj_geometry::Circle>(obs);
    std::shared_ptr<rj_geometry::Polygon> poly_obs =
        std::dynamic_pointer_cast<rj_geometry::Polygon>(obs);
    std::shared_ptr<rj_geometry::CompositeShape> comp_obs =
        std::dynamic_pointer_cast<rj_geometry::CompositeShape>(obs);
    if (circ_obs)
        draw_circle(circ_obs->center, circ_obs->radius(), color, layer);
    else if (poly_obs)
        draw_polygon(poly_obs->vertices, color, layer);
    else if (comp_obs) {
        for (const std::shared_ptr<rj_geometry::Shape>& obs : comp_obs->subshapes())
            draw_shape(obs, color, layer);
    }
}

void DebugDrawer::draw_shape_set(const rj_geometry::ShapeSet& shapes, const QColor& color,
                                 const QString& layer) {
    for (auto& shape : shapes.shapes()) {
        draw_shape(shape, color, layer);
    }
}

void DebugDrawer::draw_line(const rj_geometry::Segment& line, const QColor& qc,
                            const QString& layer) {
    Packet::DebugPath* dbg = log_frame_.add_debug_paths();
    dbg->set_layer(find_debug_layer(layer));
    *dbg->add_points() = line.pt[0];
    *dbg->add_points() = line.pt[1];
    dbg->set_color(color(qc));
}

void DebugDrawer::draw_line(rj_geometry::Point p0, rj_geometry::Point p1, const QColor& color,
                            const QString& layer) {
    draw_line(rj_geometry::Segment(p0, p1), color, layer);
}

void DebugDrawer::draw_text(const QString& text, rj_geometry::Point pos, const QColor& qc,
                            const QString& layer) {
    Packet::DebugText* dbg = log_frame_.add_debug_texts();
    dbg->set_layer(find_debug_layer(layer));
    dbg->set_text(text.toStdString());
    *dbg->mutable_pos() = pos;
    dbg->set_color(color(qc));
}

void DebugDrawer::draw_segment(const rj_geometry::Segment& line, const QColor& qc,
                               const QString& layer) {
    Packet::DebugPath* dbg = log_frame_.add_debug_paths();
    dbg->set_layer(find_debug_layer(layer));
    *dbg->add_points() = line.pt[0];
    *dbg->add_points() = line.pt[1];
    dbg->set_color(color(qc));
}