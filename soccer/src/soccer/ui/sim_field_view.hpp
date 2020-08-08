// An extension of FieldView that generates SimCommands in response
// to clicks/drags when live.

#pragma once

#include <rj_protos/grSim_Commands.pb.h>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/grSim_Replacement.pb.h>

#include <context.hpp>

#include "field_view.hpp"

class SimFieldView : public FieldView {
    Q_OBJECT;

public:
    SimFieldView(QWidget* parent = nullptr);
    void setContext(Context* context);

Q_SIGNALS:
    // Emitted when the user selects a robot.
    // The robot is identified by shell number.
    // shell may be -1 to select no robot.
    void robotSelected(int shell);

protected:
    virtual void mouseReleaseEvent(QMouseEvent*) override;
    virtual void mousePressEvent(QMouseEvent*) override;
    virtual void mouseMoveEvent(QMouseEvent*) override;

    virtual void drawTeamSpace(QPainter& p) override;

private:

    // True while a line is being dragged from the ball
    enum { DRAG_NONE = 0, DRAG_PLACE, DRAG_SHOOT } _dragMode;

    int _dragRobot;
    int _dragRobotBlue;
    Geometry2d::Point _dragTo;
    Geometry2d::Point _shot;
    Context* context_{};
};
