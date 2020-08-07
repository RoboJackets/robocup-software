#pragma once

#include <rj_protos/LogFrame.pb.h>

#include <Geometry2d/Arc.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <QColor>
#include <QMap>
#include <memory>
#include <string>
#include <vector>

class Context;

class DebugDrawer {
public:
    DebugDrawer(Context* context) : context_(context), num_debug_layers_(0) {
        reset_log_frame();
    }

    const QStringList& debug_layers() const { return debug_layers_; }

    /// Returns the number of a debug layer given its name
    int find_debug_layer(QString layer);

    /** @ingroup drawing_functions */
    void draw_polygon(const Geometry2d::Point* pts, int n,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_polygon(const std::vector<Geometry2d::Point>& pts,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_polygon(const Geometry2d::Polygon& pts,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_circle(Geometry2d::Point center, float radius,
                    const QColor& qc = Qt::black,
                    const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_arc(const Geometry2d::Arc& arc, const QColor& qw = Qt::black,
                 const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_shape(const std::shared_ptr<Geometry2d::Shape>& obs,
                   const QColor& qw = Qt::black,
                   const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_shape_set(const Geometry2d::ShapeSet& shapes,
                      const QColor& qw = Qt::black,
                      const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_line(const Geometry2d::Segment& line, const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_line(Geometry2d::Point p0, Geometry2d::Point p1,
                  const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_text(const QString& text, Geometry2d::Point pos,
                  const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void draw_segment(const Geometry2d::Segment& line,
                     const QColor& qw = Qt::black,
                     const QString& layer = QString());

    /**
     * Fill the given log frame with the current debug drawing information,
     * and reset our current debug drawing data for the next cycle.
     *
     * @param log_frame
     */
    void fill_log_frame(Packet::LogFrame* log_frame) {
        log_frame->MergeFrom(log_frame_);
        reset_log_frame();
    }

    /**
     * Helper pass-through method to create a new debug path.
     */
    Packet::DebugRobotPath* add_debug_path() {
        return log_frame_.add_debug_robot_paths();
    }

private:
    void reset_log_frame() {
        log_frame_.Clear();
        for (const QString& string : debug_layers_) {
            log_frame_.add_debug_layers(string.toStdString());
        }
    }
    /// Number of debug layers
    int num_debug_layers_;

    /// Map from debug layer name to ID
    QMap<QString, int> debug_layer_map_;

    /// Debug layers in order by ID
    QStringList debug_layers_;

    Context* context_;

    // Keep an entire log frame, but only fill the parts related to debug
    // drawing. Then we can use protobuf's merge functionality to merge it into
    // the main log frame.
    Packet::LogFrame log_frame_;
};
