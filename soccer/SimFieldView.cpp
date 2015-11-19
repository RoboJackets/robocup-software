#include <SimFieldView.hpp>

#include <Constants.hpp>
#include <Network.hpp>

#include <QPainter>
#include <QMouseEvent>
#include <QFont>

using namespace boost;
using namespace Packet;

// Converts from meters to m/s for manually shooting the ball
static const float ShootScale = 5;

SimFieldView::SimFieldView(QWidget* parent) : FieldView(parent) {
    _dragMode = DRAG_NONE;
    _dragRobot = -1;
    _dragRobotBlue = false;
}

void SimFieldView::mousePressEvent(QMouseEvent* me) {
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
            placeBall(me->pos());
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
    switch (_dragMode) {
        case DRAG_SHOOT:
            _dragTo = _worldToTeam * _screenToWorld * me->pos();
            break;

        case DRAG_PLACE:
            if (_dragRobot >= 0) {
                SimCommand cmd;
                SimCommand::Robot* r = cmd.add_robots();
                r->set_shell(_dragRobot);
                r->set_blue_team(_dragRobotBlue);
                r->mutable_pos()->CopyFrom(_screenToWorld * me->pos());
                r->mutable_vel()->set_x(0);
                r->mutable_vel()->set_y(0);
                sendSimCommand(cmd);
            } else {
                placeBall(me->pos());
            }
            break;

        default:
            break;
    }
    update();
}

void SimFieldView::mouseReleaseEvent(QMouseEvent* me) {
    if (_dragMode == DRAG_SHOOT) {
        SimCommand cmd;
        *cmd.mutable_ball_vel() = _teamToWorld.transformDirection(_shot);
        sendSimCommand(cmd);

        update();
    }

    _dragMode = DRAG_NONE;
}

void SimFieldView::placeBall(QPointF pos) {
    SimCommand cmd;
    cmd.mutable_ball_pos()->CopyFrom(_screenToWorld * pos);
    cmd.mutable_ball_vel()->set_x(0);
    cmd.mutable_ball_vel()->set_y(0);
    sendSimCommand(cmd);
}

void SimFieldView::sendSimCommand(const Packet::SimCommand& cmd) {
    std::string out;
    cmd.SerializeToString(&out);
    _simCommandSocket.writeDatagram(&out[0], out.size(),
                                    QHostAddress(QHostAddress::LocalHost),
                                    SimCommandPort);
}

void SimFieldView::drawTeamSpace(QPainter& p) {
    FieldView::drawTeamSpace(p);

    // Simulator drag-to-shoot
    std::shared_ptr<LogFrame> frame = currentFrame();
    if (_dragMode == DRAG_SHOOT && frame) {
        p.setPen(QPen(Qt::white, 0.025f)); // TODO: smaller pen?
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
