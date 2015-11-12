
#pragma once

#include <QGLWidget>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <protobuf/LogFrame.pb.h>

#include <set>
#include <memory>
#include <QLabel>

class Logger;

/** class that performs drawing of log data onto the field */
class FieldView : public QWidget {
public:
    FieldView(QWidget* parent = nullptr);

    void layerVisible(int i, bool value) {
        if (i >= 0 && i < _layerVisible.size()) {
            _layerVisible[i] = value;
        } else if (i >= _layerVisible.size()) {
            _layerVisible.resize(i + 1);
            _layerVisible[i] = value;
        }
    }

    bool layerVisible(int i) const {
        if (i < _layerVisible.size()) {
            return _layerVisible[i];
        } else {
            return false;
        }
    }

    void history(const std::vector<std::shared_ptr<Packet::LogFrame> >* value) {
        _history = value;
    }

    void rotate(int value);

    // True if this control is showing live (vs. historical) data.
    // If false, it will draw a red border.
    bool live;

    bool showRawRobots;
    bool showRawBalls;
    bool showCoords;
    bool showDotPatterns;
    bool showTeamNames;

protected:
    virtual void paintEvent(QPaintEvent* e) override;
    virtual void resizeEvent(QResizeEvent* e) override;

    virtual void drawWorldSpace(QPainter& p);
    virtual void drawTeamSpace(QPainter& p);

    void drawText(QPainter& p, QPointF pos, QString text, bool center = true);
    void drawField(QPainter& p, const Packet::LogFrame* frame);
    void drawRobot(QPainter& p, bool blueRobot, int ID, QPointF pos,
                   float theta, bool hasBall = false, bool faulty = false);
    void drawCoords(QPainter& p);

protected:
    // Returns a pointer to the most recent frame, or null if none is available.
    std::shared_ptr<Packet::LogFrame> currentFrame();

    // Coordinate transformations
    Geometry2d::TransformMatrix _screenToWorld;
    Geometry2d::TransformMatrix _worldToTeam;
    Geometry2d::TransformMatrix _teamToWorld;

    // Label used to display current coordinates of mouse
    QLabel* _posLabel;
    // Cursor used to trace mouse position when it is out of widget
    QCursor* _posCursor;

    // Rotation of the field in 90-degree increments (0 to 3).
    int _rotate;

    // How many degrees to rotate text so it shows up the right way on screen
    int _textRotation;

    const std::vector<std::shared_ptr<Packet::LogFrame> >* _history;

    QVector<bool> _layerVisible;
};
