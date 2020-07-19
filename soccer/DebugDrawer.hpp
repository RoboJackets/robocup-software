#pragma once

#include <QColor>
#include <QMap>
#include <memory>
#include <string>
#include <vector>

#include <Geometry2d/Arc.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/ShapeSet.hpp>

#include "protobuf/LogFrame.pb.h"

class Context;

class DebugDrawer {
public:
    DebugDrawer(Context* context) : _context(context), _numDebugLayers(0) {
        resetLogFrame();
    }

    const QStringList& debugLayers() const { return _debugLayers; }

    /// Returns the number of a debug layer given its name
    int findDebugLayer(QString layer);

    /** @ingroup drawing_functions */
    void drawPolygon(const Geometry2d::Point* pts, int n,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawPolygon(const std::vector<Geometry2d::Point>& pts,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawPolygon(const Geometry2d::Polygon& pts,
                     const QColor& qc = Qt::black,
                     const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawCircle(Geometry2d::Point center, float radius,
                    const QColor& qc = Qt::black,
                    const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawArc(const Geometry2d::Arc& arc, const QColor& qw = Qt::black,
                 const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawShape(const std::shared_ptr<Geometry2d::Shape>& obs,
                   const QColor& qw = Qt::black,
                   const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawShapeSet(const Geometry2d::ShapeSet& shapes,
                      const QColor& qw = Qt::black,
                      const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawLine(const Geometry2d::Segment& line, const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawLine(Geometry2d::Point p0, Geometry2d::Point p1,
                  const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawText(const QString& text, Geometry2d::Point pos,
                  const QColor& qw = Qt::black,
                  const QString& layer = QString());

    /** @ingroup drawing_functions */
    void drawSegment(const Geometry2d::Segment& line,
                     const QColor& qw = Qt::black,
                     const QString& layer = QString());

    /**
     * Fill the given log frame with the current debug drawing information,
     * and reset our current debug drawing data for the next cycle.
     *
     * @param log_frame
     */
    void fillLogFrame(Packet::LogFrame* log_frame) {
        log_frame->MergeFrom(_logFrame);
        resetLogFrame();
    }

    /**
     * Helper pass-through method to create a new debug path.
     */
    Packet::DebugRobotPath* addDebugPath() {
        return _logFrame.add_debug_robot_paths();
    }

private:
    void resetLogFrame() {
        _logFrame.Clear();
        for (const QString& string : _debugLayers) {
            _logFrame.add_debug_layers(string.toStdString());
        }
    }
    /// Number of debug layers
    int _numDebugLayers;

    /// Map from debug layer name to ID
    QMap<QString, int> _debugLayerMap;

    /// Debug layers in order by ID
    QStringList _debugLayers;

    Context* _context;

    // Keep an entire log frame, but only fill the parts related to debug
    // drawing. Then we can use protobuf's merge functionality to merge it into
    // the main log frame.
    Packet::LogFrame _logFrame;
};
