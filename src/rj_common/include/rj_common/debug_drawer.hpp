#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QColor>
#include <QMap>

#include <rj_geometry/arc.hpp>
#include <rj_geometry/composite_shape.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/polygon.hpp>
#include <rj_geometry/segment.hpp>
#include <rj_geometry/shape_set.hpp>

struct Context;

class DebugDrawer {
public:
    DebugDrawer(Context* context) : num_debug_layers_(0), context_(context) {}

    const QStringList& debug_layers() const { return debug_layers_; }

    /// Returns the number of a debug layer given its name
    int find_debug_layer(QString layer);

    /** @ingroup drawing_functions */
    void draw_polygon(const rj_geometry::Point* pts, int n,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_polygon(const std::vector<rj_geometry::Point>& pts,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_polygon(const rj_geometry::Polygon& pts,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_circle(rj_geometry::Point center, float radius,
                    const QColor& qc = Qt::black,
                    const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_arc(const rj_geometry::Arc& arc, const QColor& qw = Qt::black,
                 const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_shape(const std::shared_ptr<rj_geometry::Shape>& obs,
                   const QColor& qw = Qt::black,
                   const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_shape_set(const rj_geometry::ShapeSet& shapes,
                      const QColor& qw = Qt::black,
                      const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_line(const rj_geometry::Segment& line, const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_line(rj_geometry::Point p0, rj_geometry::Point p1,
                  const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_text(const QString& text, rj_geometry::Point pos,
                  const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_segment(const rj_geometry::Segment& line,
                     const QColor& qw = Qt::black,
                     const QString& layer = QString());

private:
    /// Number of debug layers
    int num_debug_layers_;

    /// Map from debug layer name to ID
    QMap<QString, int> debug_layer_map_;

    /// Debug layers in order by ID
    QStringList debug_layers_;

    Context* context_;
};
