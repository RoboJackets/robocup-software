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
    _dragMode = DRAG_NONE;
    _dragRobot = -1;
    _dragRobotBlue = 0;
}

void SimFieldView::setup(Context* context, rclcpp::Node* node) {
    this->context_ = context;
    _node = node;

    _sim_placement =
        _node->create_client<rj_msgs::srv::SimPlacement>(sim::topics::kSimPlacementSrv);
}

void SimFieldView::mousePressEvent(QMouseEvent* me) {
    // Ignore mouse events in the field if not in sim
    if (!context_->game_settings.simulation) {
        return;
    }

    rj_geometry::Point pos = _worldToTeam * _screenToWorld * me->pos();

    std::shared_ptr<LogFrame> frame = currentFrame();
    if (me->button() == Qt::LeftButton && frame) {
        _dragRobot = -1;
        for (const LogFrame::Robot& r : frame->self()) {
            if (pos.near_point(r.pos(), kRobotRadius)) {
                _dragRobot = r.shell();
                _dragRobotBlue = frame->blue_team();
                break;
            }
        }
        for (const LogFrame::Robot& r : frame->opp()) {
            if (pos.near_point(r.pos(), kRobotRadius)) {
                _dragRobot = r.shell();
                _dragRobotBlue = !frame->blue_team();
                break;
            }
        }

        if (_dragRobot < 0) {
            dragBall(me->pos());
        }

        _dragMode = DRAG_PLACE;
    } else if (me->button() == Qt::RightButton && frame) {
        if (frame->has_ball() && pos.near_point(frame->ball().pos(), 10 * kBallRadius)) {
            // Drag to shoot the ball
            _dragMode = DRAG_SHOOT;
            _dragTo = pos;
        } else {
            // Look for a robot selection
            int newID = -1;
            for (int i = 0; i < frame->self_size(); ++i) {
                if (pos.dist_to(frame->self(i).pos()) < kRobotRadius) {
                    newID = frame->self(i).shell();
                    break;
                }
            }

            if (newID != frame->manual_id()) {
                robotSelected(newID);
            }
        }
    }
}

void SimFieldView::mouseMoveEvent(QMouseEvent* me) {
    FieldView::mouseMoveEvent(me);
    switch (_dragMode) {
        case DRAG_SHOOT:
            _dragTo = _worldToTeam * _screenToWorld * me->pos();
            break;

        case DRAG_PLACE:
            if (_dragRobot >= 0) {
                dragRobot(me->pos(), _dragRobot);
            } else {
                dragBall(me->pos());
            }
            break;

        default:
            break;
    }
    update();
}

void SimFieldView::dragBall(const QPoint& screen_pos) {
    auto request = std::make_shared<rj_msgs::srv::SimPlacement::Request>();
    request->ball.position.push_back(rj_convert::convert_to_ros(_screenToWorld * screen_pos));
    _sim_placement->async_send_request(request);
}

void SimFieldView::kickBall(const rj_geometry::Point& shot) {
    auto request = std::make_shared<rj_msgs::srv::SimPlacement::Request>();
    request->ball.velocity.push_back(
        rj_convert::convert_to_ros(_teamToWorld.transform_direction(shot)));
    _sim_placement->async_send_request(request);
}

void SimFieldView::dragRobot(const QPoint& screen_pos, int robot_id) {
    auto request = std::make_shared<rj_msgs::srv::SimPlacement::Request>();
    rj_msgs::msg::RobotPlacement robot;
    robot.pose.position = rj_convert::convert_to_ros(_screenToWorld * screen_pos);
    robot.robot_id = _dragRobot;
    robot.is_blue_team = _dragRobotBlue;
    request->robots.emplace_back(robot);
    _sim_placement->async_send_request(request);
}

void SimFieldView::mouseReleaseEvent(QMouseEvent* /*me*/) {
    if (_dragMode == DRAG_SHOOT) {
        grSim_Packet simPacket;
        grSim_BallReplacement* ball_replace = simPacket.mutable_replacement()->mutable_ball();

        ball_replace->set_vx(_teamToWorld.transform_direction(_shot).x());
        ball_replace->set_vy(_teamToWorld.transform_direction(_shot).y());
        context_->grsim_command = simPacket;
        kickBall(_shot);

        update();
    }

    _dragMode = DRAG_NONE;
}

void SimFieldView::drawTeamSpace(QPainter& p) {
    FieldView::drawTeamSpace(p);

    // Simulator drag-to-shoot
    std::shared_ptr<LogFrame> frame = currentFrame();
    if (_dragMode == DRAG_SHOOT && frame) {
        p.setPen(QPen(Qt::white, 0.025f));
        rj_geometry::Point ball = frame->ball().pos();
        p.drawLine(ball.to_q_point_f(), _dragTo.to_q_point_f());

        if (ball != _dragTo) {
            p.setPen(QPen(Qt::gray, 0.025f));

            _shot = (ball - _dragTo) * ShootScale;
            float speed = _shot.mag();
            rj_geometry::Point shotExtension = ball + _shot / speed * 8;

            p.drawLine(ball.to_q_point_f(), shotExtension.to_q_point_f());

            p.setPen(Qt::black);
            QFont font;
            font.setPixelSize(30);
            p.setFont(font);
            drawText(p, _dragTo.to_q_point_f(), QString("%1 m/s").arg(speed, 0, 'f', 1));
        }
    }
}
