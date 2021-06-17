#include "sim_field_view.hpp"

#include <QFont>
#include <QMouseEvent>
#include <QPainter>

#include <rj_common/network.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/ball_placement.hpp>
#include <rj_msgs/msg/robot_placement.hpp>

using namespace boost;
using namespace Packet;

// Converts from meters to m/s for manually shooting the ball
static const float ShootScale = 5;

SimFieldView::SimFieldView(QWidget* parent) : FieldView(parent) {
    drag_mode_ = DRAG_NONE;
    drag_robot_ = -1;
    drag_robot_blue_ = false;
}

void SimFieldView::setup(Context* context, rclcpp::Node* node) {
    this->context_ = context;
    node_ = node;

    sim_placement_ =
        node_->create_client<rj_msgs::srv::SimPlacement>(sim::topics::kSimPlacementSrv);
}

void SimFieldView::mousePressEvent(QMouseEvent* me) {
    // Ignore mouse events in the field if not in sim
    if (!context_->game_settings.simulation) {
        return;
    }

    rj_geometry::Point pos = _worldToTeam * _screenToWorld * me->pos();

    std::shared_ptr<LogFrame> frame = currentFrame();
    if (me->button() == Qt::LeftButton && frame) {
        drag_robot_ = -1;
        for (const LogFrame::Robot& r : frame->self()) {
            if (pos.near_point(r.pos(), kRobotRadius)) {
                drag_robot_ = r.shell();
                drag_robot_blue_ = frame->blue_team();
                break;
            }
        }
        for (const LogFrame::Robot& r : frame->opp()) {
            if (pos.near_point(r.pos(), kRobotRadius)) {
                drag_robot_ = r.shell();
                drag_robot_blue_ = !frame->blue_team();
                break;
            }
        }

        if (drag_robot_ < 0) {
            drag_ball(me->pos());
        }

        drag_mode_ = DRAG_PLACE;
    } else if (me->button() == Qt::RightButton && frame) {
        if (frame->has_ball() && pos.near_point(frame->ball().pos(), 10 * kBallRadius)) {
            // Drag to shoot the ball
            drag_mode_ = DRAG_SHOOT;
            drag_point_ = pos;
        } else {
            // Look for a robot selection
            int new_id = -1;
            for (int i = 0; i < frame->self_size(); ++i) {
                if (pos.dist_to(frame->self(i).pos()) < kRobotRadius) {
                    new_id = frame->self(i).shell();
                    break;
                }
            }

            if (new_id != frame->manual_id()) {
                robotSelected(new_id);
            }
        }
    }
}

void SimFieldView::mouseMoveEvent(QMouseEvent* me) {
    FieldView::mouseMoveEvent(me);
    switch (drag_mode_) {
        case DRAG_SHOOT:
            drag_point_ = _worldToTeam * _screenToWorld * me->pos();
            break;

        case DRAG_PLACE:
            if (drag_robot_ >= 0) {
                drag_robot(me->pos(), drag_robot_, drag_robot_blue_);
            } else {
                drag_ball(me->pos());
            }
            break;

        default:
            break;
    }
    update();
}

void SimFieldView::drag_ball(const QPoint& screen_pos) {
    set_ball_position(_screenToWorld * screen_pos);
}

void SimFieldView::set_ball_position(const rj_geometry::Point& field_pos) {
    auto request = std::make_shared<rj_msgs::srv::SimPlacement::Request>();
    request->ball.position.push_back(rj_convert::convert_to_ros(field_pos));
    sim_placement_->async_send_request(request);
}

void SimFieldView::set_ball_velocity(const rj_geometry::Point& shot) {
    auto request = std::make_shared<rj_msgs::srv::SimPlacement::Request>();
    request->ball.velocity.push_back(
        rj_convert::convert_to_ros(_teamToWorld.transform_direction(shot)));
    sim_placement_->async_send_request(request);
}

void SimFieldView::drag_robot(const QPoint& screen_pos, int robot_id, bool is_blue) {
    set_robot_pose(rj_geometry::Pose(_screenToWorld * screen_pos, 0), robot_id, is_blue);
}

void SimFieldView::set_robot_pose(const rj_geometry::Pose& field_pose, int robot_id, bool is_blue) {
    auto request = std::make_shared<rj_msgs::srv::SimPlacement::Request>();
    rj_msgs::msg::RobotPlacement robot;
    robot.pose = rj_convert::convert_to_ros(field_pose);
    robot.robot_id = robot_id;
    robot.is_blue_team = is_blue;
    request->robots.emplace_back(robot);
    sim_placement_->async_send_request(request);
}

void SimFieldView::mouseReleaseEvent(QMouseEvent* /*me*/) {
    if (drag_mode_ == DRAG_SHOOT) {
        set_ball_velocity(shot_);
        update();
    }

    drag_mode_ = DRAG_NONE;
}

void SimFieldView::drawTeamSpace(QPainter& p) {
    FieldView::drawTeamSpace(p);

    // Simulator drag-to-shoot
    std::shared_ptr<LogFrame> frame = currentFrame();
    if (drag_mode_ == DRAG_SHOOT && frame) {
        p.setPen(QPen(Qt::white, 0.025f));
        rj_geometry::Point ball = frame->ball().pos();
        p.drawLine(ball.to_q_point_f(), drag_point_.to_q_point_f());

        if (ball != drag_point_) {
            p.setPen(QPen(Qt::gray, 0.025f));

            shot_ = (ball - drag_point_) * ShootScale;
            double speed = shot_.mag();
            rj_geometry::Point shot_extension = ball + shot_ / speed * 8;

            p.drawLine(ball.to_q_point_f(), shot_extension.to_q_point_f());

            p.setPen(Qt::black);
            QFont font;
            font.setPixelSize(30);
            p.setFont(font);
            drawText(p, drag_point_.to_q_point_f(), QString("%1 m/s").arg(speed, 0, 'f', 1));
        }
    }
}
