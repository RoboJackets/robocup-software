#include "rj_common/debug_drawer.hpp"

#include <rj_geometry/circle.hpp>
#include <rj_geometry/rect.hpp>

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
    DebugPath dbg;
    dbg.layer = find_debug_layer(layer);
    dbg.color = color(qc);
    dbg.points.reserve(n);
    for (int i = 0; i < n; ++i) {
        dbg.points.push_back(pts[i]);
    }
    current_.polygons.push_back(std::move(dbg));
}

void DebugDrawer::draw_polygon(const std::vector<rj_geometry::Point>& pts, const QColor& qc,
                               const QString& layer) {
    draw_polygon(pts.data(), static_cast<int>(pts.size()), qc, layer);
}

void DebugDrawer::draw_polygon(const rj_geometry::Polygon& polygon, const QColor& qc,
                               const QString& layer) {
    this->draw_polygon(polygon.vertices, qc, layer);
}

void DebugDrawer::draw_circle(rj_geometry::Point center, float radius, const QColor& qc,
                              const QString& layer) {
    DebugCircle dbg;
    dbg.layer = find_debug_layer(layer);
    dbg.center = center;
    dbg.radius = radius;
    dbg.color = color(qc);
    current_.circles.push_back(dbg);
}

void DebugDrawer::draw_arc(const rj_geometry::Arc& arc, const QColor& qc, const QString& layer) {
    DebugArc dbg;
    dbg.layer = find_debug_layer(layer);
    dbg.center = arc.center();
    dbg.radius = static_cast<float>(arc.radius());
    dbg.start = static_cast<float>(arc.start());
    dbg.end = static_cast<float>(arc.end());
    dbg.color = color(qc);
    current_.arcs.push_back(dbg);
}

void DebugDrawer::draw_shape(const std::shared_ptr<rj_geometry::Shape>& obs, const QColor& color,
                             const QString& layer) {
    std::shared_ptr<rj_geometry::Circle> circ_obs =
        std::dynamic_pointer_cast<rj_geometry::Circle>(obs);
    std::shared_ptr<rj_geometry::Polygon> poly_obs =
        std::dynamic_pointer_cast<rj_geometry::Polygon>(obs);
    std::shared_ptr<rj_geometry::CompositeShape> comp_obs =
        std::dynamic_pointer_cast<rj_geometry::CompositeShape>(obs);
    std::shared_ptr<rj_geometry::Rect> rect_obs = std::dynamic_pointer_cast<rj_geometry::Rect>(obs);
    if (circ_obs) {
        draw_circle(circ_obs->center, circ_obs->radius(), color, layer);
    } else if (poly_obs) {
        draw_polygon(poly_obs->vertices, color, layer);
    } else if (rect_obs) {
        std::vector<rj_geometry::Point> points = {
            rj_geometry::Point{rect_obs->minx(), rect_obs->miny()},
            rj_geometry::Point{rect_obs->maxx(), rect_obs->miny()},
            rj_geometry::Point{rect_obs->maxx(), rect_obs->maxy()},
            rj_geometry::Point{rect_obs->minx(), rect_obs->maxy()}};
        draw_polygon(points.data(), static_cast<int>(points.size()), color, layer);
    } else if (comp_obs) {
        for (const std::shared_ptr<rj_geometry::Shape>& subshape : comp_obs->subshapes()) {
            draw_shape(subshape, color, layer);
        }
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
    DebugPath dbg;
    dbg.layer = find_debug_layer(layer);
    dbg.points.push_back(line.pt[0]);
    dbg.points.push_back(line.pt[1]);
    dbg.color = color(qc);
    current_.paths.push_back(std::move(dbg));
}

void DebugDrawer::draw_line(rj_geometry::Point p0, rj_geometry::Point p1, const QColor& color,
                            const QString& layer) {
    draw_line(rj_geometry::Segment(p0, p1), color, layer);
}

void DebugDrawer::draw_text(const QString& text, rj_geometry::Point pos, const QColor& qc,
                            const QString& layer) {
    DebugText dbg;
    dbg.layer = find_debug_layer(layer);
    dbg.text = text.toStdString();
    dbg.pos = pos;
    dbg.color = color(qc);
    current_.texts.push_back(std::move(dbg));
}

void DebugDrawer::draw_segment(const rj_geometry::Segment& line, const QColor& qc,
                               const QString& layer) {
    draw_line(line, qc, layer);
}
