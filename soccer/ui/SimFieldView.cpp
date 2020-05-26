#include "SimFieldView.hpp"

#include <Constants.hpp>
#include <Network.hpp>

#include <QFont>
#include <QMouseEvent>
#include <QPainter>

using namespace boost;
using namespace Packet;

// Converts from meters to m/s for manually shooting the ball
static const float ShootScale = 5;

SimFieldView::SimFieldView(QWidget* parent) : FieldView(parent) {
    _dragMode = DRAG_NONE;
    _dragRobot = -1;
    _dragRobotBlue = 0;
}

void SimFieldView::setContext(Context* context) { this->_context = context; }

void SimFieldView::mousePressEvent(QMouseEvent* me) {
    // Ignore mouse events in the field if not in sim
    if (!_context->game_settings.simulation) {
        return;
    }

    Geometry2d::Point pos = _worldToTeam * _screenToWorld * me->pos();

    std::shared_ptr<LogFrame> frame = currentFrame();
    if (me->button() == Qt::LeftButton && frame) {
        _dragRobot = -1;
        for (const LogFrame::Robot& r : frame->self()) {
            if (pos.nearPoint(r.pos(), Robot_Radius)) {
                _dragRobot = r.shell();
                _dragRobotBlue = frame->blue_team();
                break;
            }
        }
        for (const LogFrame::Robot& r : frame->opp()) {
            if (pos.nearPoint(r.pos(), Robot_Radius)) {
                _dragRobot = r.shell();
                _dragRobotBlue = !frame->blue_team();
                break;
            }
        }

        if (_dragRobot < 0) {
            _context->ball_command = me->pos();
            _context->screen_to_world_command = _screenToWorld;
        }

        _dragMode = DRAG_PLACE;
    } else if (me->button() == Qt::RightButton && frame) {
        if (frame->has_ball() &&
            pos.nearPoint(frame->ball().pos(), 10 * Ball_Radius)) {
            // Drag to shoot the ball
            _dragMode = DRAG_SHOOT;
            _dragTo = pos;
        } else {
            // Look for a robot selection
            int newID = -1;
            for (int i = 0; i < frame->self_size(); ++i) {
                if (pos.distTo(frame->self(i).pos()) < Robot_Radius) {
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
                grSim_Packet simPacket;
                grSim_RobotReplacement* robot_replace =
                    simPacket.mutable_replacement()->add_robots();

                robot_replace->set_x((_screenToWorld * me->pos()).x());
                robot_replace->set_y((_screenToWorld * me->pos()).y());
                robot_replace->set_id(_dragRobot);
                robot_replace->set_yellowteam(_dragRobotBlue == 0);
                robot_replace->set_dir(0.0);

                _context->grsim_command = simPacket;
            } else {
                _context->ball_command = me->pos();
                _context->screen_to_world_command = _screenToWorld;
            }
            break;

        default:
            break;
    }
    update();
}

void SimFieldView::mouseReleaseEvent(QMouseEvent* /*me*/) {
    if (_dragMode == DRAG_SHOOT) {
        grSim_Packet simPacket;
        grSim_BallReplacement* ball_replace =
            simPacket.mutable_replacement()->mutable_ball();

        ball_replace->set_vx(_teamToWorld.transformDirection(_shot).x());
        ball_replace->set_vy(_teamToWorld.transformDirection(_shot).y());
        _context->grsim_command = simPacket;

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
        Geometry2d::Point ball = frame->ball().pos();
        p.drawLine(ball.toQPointF(), _dragTo.toQPointF());

        if (ball != _dragTo) {
            p.setPen(QPen(Qt::gray, 0.025f));

            _shot = (ball - _dragTo) * ShootScale;
            float speed = _shot.mag();
            Geometry2d::Point shotExtension = ball + _shot / speed * 8;

            p.drawLine(ball.toQPointF(), shotExtension.toQPointF());

            p.setPen(Qt::black);
            QFont font;
            font.setPixelSize(30);
            p.setFont(font);
            drawText(p, _dragTo.toQPointF(),
                     QString("%1 m/s").arg(speed, 0, 'f', 1));
        }
    }
}
