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
    Q_OBJECT

public:
    SimFieldView(QWidget* parent);
    void setup(Context* context, rclcpp::Node* node);

    /// Set the ball's position, in world coordinates.
    void set_ball_position(const rj_geometry::Point& field_pos);
    /// Set the ball's velocity, in world coordinates.
    void set_ball_velocity(const rj_geometry::Point& shot);
    /// Set a robot's pose, in world coordinates.
    void set_robot_pose(const rj_geometry::Pose& field_pose, int robot_id, bool is_blue);

    // Drag a ball based on screen coordinates.
    void drag_ball(const QPoint& screen_pos);
    // Drag a robot based on screen coordinates.
    void drag_robot(const QPoint& screen_pos, int robot_id, bool is_blue);

Q_SIGNALS:
    // Emitted when the user selects a robot.
    // The robot is identified by shell number.
    // shell may be -1 to select no robot.
    void robotSelected(int shell);

protected:
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

    void drawTeamSpace(QPainter& p) override;

private:
    // True while a line is being dragged from the ball
    enum { DRAG_NONE = 0, DRAG_PLACE, DRAG_SHOOT } drag_mode_ = DRAG_NONE;
    int drag_robot_ = -1;
    bool drag_robot_blue_ = false;

    rj_geometry::Point drag_point_;
    rj_geometry::Point shot_;
    Context* context_{};
    rclcpp::Node* node_ = nullptr;
    rclcpp::Client<rj_msgs::srv::SimPlacement>::SharedPtr sim_placement_;
};
