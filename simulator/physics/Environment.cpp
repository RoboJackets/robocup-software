#include "Environment.hpp"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Field.hpp"
#include "Robot.hpp"
#include <Constants.hpp>
#include <Network.hpp>
#include <Geometry2d/Util.hpp>

#include <protobuf/messages_robocup_ssl_detection.pb.h>
#include <protobuf/messages_robocup_ssl_geometry.pb.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>

#include <QDomDocument>
#include <QDomAttr>
#include <QDebug>
#include <QFile>
#include <stdexcept>
#include <iostream>
#include <sys/time.h>

using namespace std;
using namespace Geometry2d;
using namespace Packet;

static const QHostAddress LocalAddress(QHostAddress::LocalHost);
static const QHostAddress MulticastAddress(SharedVisionAddress);

const int Oversample = 1;

Environment::Environment(const QString& configFile, bool sendShared_,
                         SimEngine* engine)
    : _dropFrame(false),
      _configFile(configFile),
      _frameNumber(0),
      _stepCount(0),
      _simEngine(engine),
      sendShared(sendShared_),
      ballVisibility(100) {
    // NOTE: does not start simulation/thread until triggered
    _field = new Field(this);
    _field->initPhysics();
    loadConfigFile(_configFile);
}

Environment::~Environment() { delete _field; }

void Environment::connectSockets() {
    // Bind sockets
    bool success = (_visionSocket.bind(SimCommandPort) &&
                    _radioSocketYellow.bind(RadioTxPort) &&
                    _radioSocketBlue.bind(RadioTxPort + 1));
    if (!success) {
        throw std::runtime_error(
            "Unable to bind sockets.  Is there another instance of simulator "
            "already running?");
    }

    gettimeofday(&_lastStepTime, nullptr);

    connect(&_timer, SIGNAL(timeout()), SLOT(step()));
    _timer.start(16 / Oversample);
}

void Environment::preStep(float deltaTime) {
    for (Robot* robot : _yellow) {
        robot->applyEngineForces(deltaTime);
    }

    for (Robot* robot : _blue) {
        robot->applyEngineForces(deltaTime);
    }
}

void Environment::step() {
    // Check for SimCommands
    while (_visionSocket.hasPendingDatagrams()) {
        SimCommand cmd;
        if (!loadPacket<SimCommand>(_visionSocket, cmd)) continue;
        handleSimCommand(cmd);
    }

    // Check for RadioTx packets from blue team
    while (_radioSocketBlue.hasPendingDatagrams()) {
        RadioTx tx;
        if (!loadPacket<RadioTx>(_radioSocketBlue, tx)) continue;

        handleRadioTx(true, tx);
    }

    // Check for RadioTx packets from yellow team
    while (_radioSocketYellow.hasPendingDatagrams()) {
        RadioTx tx;
        if (!loadPacket<RadioTx>(_radioSocketYellow, tx)) continue;
        handleRadioTx(false, tx);
    }

    // timing
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    _lastStepTime = tv;

    // execute simulation step
    // TODO: execute

    // Send vision data
    ++_stepCount;
    if (_stepCount == Oversample) {
        _stepCount = 0;

        if (_dropFrame) {
            _dropFrame = false;
        } else {
            sendVision();
        }
    }
}

void Environment::handleSimCommand(const Packet::SimCommand& cmd) {
    if (!_balls.empty()) {
        if (cmd.has_ball_vel()) {
            _balls[0]->velocity(cmd.ball_vel().x(), cmd.ball_vel().y());
        }
        if (cmd.has_ball_pos()) {
            _balls[0]->position(cmd.ball_pos().x(), cmd.ball_pos().y());
        }
    }

    for (const SimCommand::Robot& rcmd : cmd.robots()) {
        const RobotMap& team = rcmd.blue_team() ? _blue : _yellow;
        RobotMap::const_iterator i = team.find(rcmd.shell());

        if (i == team.end()) {
            if (rcmd.has_pos()) {
                // add a new robot
                addRobot(
                    rcmd.blue_team(), rcmd.shell(), rcmd.pos(),
                    Robot::rev2008);  // TODO: make this check robot revision
            } else {
                // if there's no position, we can't add a robot
                printf("Trying to override non-existent robot %d:%d\n",
                       rcmd.blue_team(), rcmd.shell());
                continue;
            }
        }

        // remove a robot if it is marked not visible
        if (rcmd.has_visible() && !rcmd.visible()) {
            removeRobot(rcmd.blue_team(), rcmd.shell());
            continue;
        }

        // change existing robots
        Robot* robot = *i;

        if (rcmd.has_pos()) {
            float x = rcmd.pos().x();
            float y = rcmd.pos().y();
            robot->position(x, y);

            QVector3D pos3(x, y, 0.0);
            QVector3D axis(0.0, 0.0, 1.0);
        }

        float new_w = 0.0;
        if (rcmd.has_w()) {
            new_w = rcmd.w();
            if (!rcmd.has_vel()) {
                robot->velocity(0.0, 0.0, new_w);
            }
        }

        if (rcmd.has_vel()) {
            robot->velocity(rcmd.vel().x(), rcmd.vel().y(), new_w);
        }
    }

    if (cmd.has_reset() && cmd.reset()) {
        resetScene();
    }
}

void Environment::sendVision() {
    SSL_WrapperPacket wrapper;
    SSL_DetectionFrame* det = wrapper.mutable_detection();
    det->set_frame_number(_frameNumber++);
    det->set_camera_id(0);

    struct timeval tv;
    gettimeofday(&tv, nullptr);
    det->set_t_capture(tv.tv_sec + (double)tv.tv_usec * 1.0e-6);
    det->set_t_sent(det->t_capture());

    for (Robot* robot : _yellow) {
        if ((rand() % 100) < robot->visibility) {
            SSL_DetectionRobot* out = det->add_robots_yellow();
            convert_robot(robot, out);
        }
    }

    for (Robot* robot : _blue) {
        if ((rand() % 100) < robot->visibility) {
            SSL_DetectionRobot* out = det->add_robots_blue();
            convert_robot(robot, out);
        }
    }

    for (const Ball* b : _balls) {
        Geometry2d::Point ballPos = b->getPosition();

        // bool occ;
        // if (ballPos.x < 0)
        // {
        // 	occ = occluded(ballPos, cam0);
        // } else {
        // 	occ = occluded(ballPos, cam1);
        // }

        if ((rand() % 100) < ballVisibility) {
            SSL_DetectionBall* out = det->add_balls();
            out->set_confidence(1);
            out->set_x(ballPos.x * 1000);
            out->set_y(ballPos.y * 1000);
            out->set_pixel_x(ballPos.x * 1000);
            out->set_pixel_y(ballPos.y * 1000);
        }
    }

    std::string buf;
    wrapper.SerializeToString(&buf);

    if (sendShared) {
        _visionSocket.writeDatagram(&buf[0], buf.size(), MulticastAddress,
                                    SharedVisionPort);
    } else {
        _visionSocket.writeDatagram(&buf[0], buf.size(), LocalAddress,
                                    SimVisionPort);
        _visionSocket.writeDatagram(&buf[0], buf.size(), LocalAddress,
                                    SimVisionPort + 1);
    }
}

void Environment::convert_robot(const Robot* robot, SSL_DetectionRobot* out) {
    Geometry2d::Point pos = robot->getPosition();
    out->set_confidence(1);
    out->set_robot_id(robot->shell);
    out->set_x(pos.x * 1000);
    out->set_y(pos.y * 1000);
    out->set_orientation(robot->getAngle());
    out->set_pixel_x(pos.x * 1000);
    out->set_pixel_y(pos.y * 1000);
}

void Environment::addBall(Geometry2d::Point pos) {
    Ball* b = new Ball(this);
    b->initPhysics();
    b->position(pos.x, pos.y);

    _balls.append(b);

    printf("New Ball: %f %f\n", pos.x, pos.y);
}

void Environment::addRobot(bool blue, int id, Geometry2d::Point pos,
                           Robot::RobotRevision rev) {
    Robot* r = new Robot(this, id, rev, pos);
    r->initPhysics(blue);

    if (blue) {
        _blue.insert(id, r);
    } else {
        _yellow.insert(id, r);
    }

    Geometry2d::Point actPos = r->getPosition();

    switch (rev) {
        case Robot::rev2008:
            printf("New 2008 Robot: %d : %f %f\n", id, actPos.x, actPos.y);
            break;
        case Robot::rev2011:
            printf("New 2011 Robot: %d : %f %f\n", id, actPos.x, actPos.y);
    }

    QVector3D pos3(pos.x, pos.y, 0.0);
    QVector3D axis(0.0, 0.0, 1.0);
}

void Environment::removeRobot(bool blue, int id) {
    if (blue) {
        _blue.remove(id);
    } else {
        _yellow.remove(id);
    }
}

Geometry2d::Point gaussianPoint(int n, float scale) {
    Geometry2d::Point pt;
    for (int i = 0; i < n; ++i) {
        pt.x += drand48() - 0.5;
        pt.y += drand48() - 0.5;
    }
    pt *= scale / n;

    return pt;
}

bool Environment::occluded(Geometry2d::Point ball, Geometry2d::Point camera) {
    float camZ = 4;
    float ballZ = Ball_Radius;
    float intZ = Robot_Height;

    // FIXME: use actual physics engine to determine line of sight

    // Find where the line from the camera to the ball intersects the
    // plane at the top of the robots.
    //
    // intZ = (ballZ - camZ) * t + camZ
    float t = (intZ - camZ) / (ballZ - camZ);
    Geometry2d::Point intersection;
    intersection.x = (ball.x - camera.x) * t + camera.x;
    intersection.y = (ball.y - camera.y) * t + camera.y;

    // Return true if the intersection point is inside any robot
    for (const Robot* r : _blue) {
        if (intersection.nearPoint(r->getPosition(), Robot_Radius)) {
            return true;
        }
    }
    for (const Robot* r : _yellow) {
        if (intersection.nearPoint(r->getPosition(), Robot_Radius)) {
            return true;
        }
    }
    return false;
}

Robot* Environment::robot(bool blue, int board_id) const {
    const QMap<unsigned int, Robot*>& robots = blue ? _blue : _yellow;

    if (robots.contains(board_id)) {
        return robots.value(board_id, nullptr);
    } else {
        return nullptr;
    }
}

void Environment::handleRadioTx(bool blue, const Packet::RadioTx& tx) {
    for (int i = 0; i < tx.robots_size(); ++i) {
        const Packet::RadioTx::Robot& cmd = tx.robots(i);

        Robot* r = robot(blue, cmd.robot_id());
        if (r) {
            // run controls update
            r->radioTx(&cmd);

            // trigger signals to update visualization
            Geometry2d::Point pos2 = r->getPosition();
            QVector3D pos3(pos2.x, pos2.y, 0.0);
            QVector3D axis(0.0, 0.0, 1.0);
        } else {
            printf("Commanding nonexistent robot %s:%d\n",
                   blue ? "Blue" : "Yellow", cmd.robot_id());
        }

        Packet::RadioRx rx = r->radioRx();
        rx.set_robot_id(r->shell);

        // Send the RX packet
        std::string out;
        rx.SerializeToString(&out);
        if (blue)
            _radioSocketBlue.writeDatagram(&out[0], out.size(), LocalAddress,
                                           RadioRxPort + 1);
        else
            _radioSocketYellow.writeDatagram(&out[0], out.size(), LocalAddress,
                                             RadioRxPort);
    }

    // FIXME: the interface changed for this part
    // Robot *rev = robot(blue, tx.robot_id());
    // if (rev)
    // {
    // 	Packet::RadioRx rx = rev->radioRx();
    // 	rx.set_robot_id(tx.robot_id());
    //
    // 	// Send the RX packet
    // 	std::string out;
    // 	rx.SerializeToString(&out);
    // 	_radioSocket[ch].writeDatagram(&out[0], out.size(), LocalAddress,
    // RadioRxPort + ch);
    // }
}

void Environment::renderScene(GL_ShapeDrawer* shapeDrawer,
                              const btVector3& worldBoundsMin,
                              const btVector3& worldBoundsMax) {
    _field->renderField();
    for (Robot* r : _blue) {
        r->renderWheels(shapeDrawer, worldBoundsMin, worldBoundsMax);
    }
    for (Robot* r : _yellow) {
        r->renderWheels(shapeDrawer, worldBoundsMin, worldBoundsMax);
    }
}

void Environment::resetScene() {
    for (Robot* r : _blue) {
        r->resetScene();
    }
    for (Robot* r : _yellow) {
        r->resetScene();
    }
    for (Ball* b : _balls) {
        b->resetScene();
    }
}

bool Environment::loadConfigFile() { return loadConfigFile(_configFile); }

bool Environment::loadConfigFile(const QString& filename) {
    // load the config file
    QFile configFile(filename);

    if (!configFile.exists()) {
        fprintf(stderr, "Configuration file %s does not exist\n",
                (const char*)filename.toLatin1());
        return false;
    }

    QDomDocument _doc;

    if (!configFile.open(QIODevice::ReadOnly)) {
        throw std::runtime_error("Unable to open config file.");
    }

    if (!_doc.setContent(&configFile)) {
        configFile.close();
        throw std::runtime_error("Internal: unable to set document content.");
    }
    configFile.close();

    // load rest of file
    qDebug() << "Loading config: " << filename;

    QDomElement root = _doc.documentElement();

    if (root.isNull() || root.tagName() != QString("simulation")) {
        throw std::runtime_error("Document format: expected <motion> tag");
    }

    QDomElement element = root.firstChildElement();

    while (!element.isNull()) {
        if (element.tagName() == QString("ball")) {
            float x = element.attribute("x").toFloat();
            float y = element.attribute("y").toFloat();

            addBall(Geometry2d::Point(x, y));
        } else if (element.tagName() == QString("blue")) {
            procTeam(element, true);
        } else if (element.tagName() == QString("yellow")) {
            procTeam(element, false);
        }

        element = element.nextSiblingElement();
    }

    return true;
}

void Environment::procTeam(QDomElement e, bool blue) {
    QDomElement elem = e.firstChildElement();

    while (!elem.isNull()) {
        if (elem.tagName() == "robot") {
            float x = elem.attribute("x").toFloat();
            float y = elem.attribute("y").toFloat();
            int id = elem.attribute("id").toInt();

            if (elem.hasAttribute("rev")) {
                QString rev = elem.attribute("rev");
                Robot::RobotRevision r = Robot::rev2008;
                if (rev.contains("2008")) {
                    r = Robot::rev2008;
                } else if (rev.contains("2011")) {
                    r = Robot::rev2011;
                }
                addRobot(blue, id, Geometry2d::Point(x, y), r);
            } else {
                addRobot(blue, id, Geometry2d::Point(x, y), Robot::rev2008);
            }
        }

        elem = elem.nextSiblingElement();
    }
}

void Environment::reshapeFieldBodies() { _field->reshapeBodies(); }
