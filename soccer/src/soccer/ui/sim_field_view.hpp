// An extension of FieldView that generates SimCommands in response
// to clicks/drags when live.

#pragma once

#include <context.hpp>
#include <rj_msgs/srv/sim_placement.hpp>
#include <rj_protos/grSim_Commands.pb.h>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/grSim_Replacement.pb.h>

#include "field_view.hpp"

class SimFieldView : public FieldView {
    Q_OBJECT;

public:
    SimFieldView(QWidget* parent);
    void setup(Context* context, rclcpp::Node* node);

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
    void dragBall(const QPoint& screen_pos);
    void kickBall(const rj_geometry::Point& shot);
    void dragRobot(const QPoint& screen_pos, int robot_id);

    // True while a line is being dragged from the ball
    enum { DRAG_NONE = 0, DRAG_PLACE, DRAG_SHOOT } _dragMode;

    int _dragRobot;
    int _dragRobotBlue;
    rj_geometry::Point _dragTo;
    rj_geometry::Point _shot;
    Context* context_{};
    rclcpp::Node* _node = nullptr;
    rclcpp::Client<rj_msgs::srv::SimPlacement>::SharedPtr _sim_placement;
};
