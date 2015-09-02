
#include <FieldView.hpp>

#include <stdio.h>

#include <Network.hpp>
#include <LogUtils.hpp>
#include <Constants.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Util.hpp>

#include <QStyleOption>
#include <QLayout>
#include <QPainter>
#include <QResizeEvent>
#include <algorithm>
#include <sys/socket.h>

#include <VisionDotPattern.hpp>

using namespace std;
using namespace boost;
using namespace Packet;

static QPen redPen(Qt::red, 0);
static QPen bluePen(Qt::blue, 0);
static QPen yellowPen(Qt::yellow, 0);
static QPen blackPen(Qt::black, 0);
static QPen whitePen(Qt::white, 0);
static QPen greenPen(Qt::green, 0);
static QPen grayPen(Qt::gray, 0);
static QPen darkRedPen(Qt::darkRed, 0);

static QPen tempPen(Qt::white, 0);

static QColor ballColor(0xff, 0x90, 0);
static QPen ballPen(ballColor, 0);

FieldView::FieldView(QWidget* parent) : QWidget(parent) {
    showRawRobots = false;
    showRawBalls = false;
    showCoords = false;
    showDotPatterns = false;
    showTeamNames = false;
    _rotate = 1;
    _history = nullptr;

    // Green background
    QPalette p = palette();
    p.setColor(QPalette::Window, QColor(0, 85.0, 0));
    setPalette(p);
    setAutoFillBackground(true);
}

std::shared_ptr<LogFrame> FieldView::currentFrame() {
    if (_history && !_history->empty()) {
        return _history->at(0);
    } else {
        return std::shared_ptr<LogFrame>();
    }
}

void FieldView::rotate(int value) {
    _rotate = value;

    // Fix size
    updateGeometry();

    update();
}

void FieldView::paintEvent(QPaintEvent* e) {
    QPainter p(this);

    // antialiasing drastically improves rendering quality
    p.setRenderHint(QPainter::Antialiasing);

    if (!live) {
        // Non-live border
        p.setPen(QPen(Qt::red, 4));
        p.drawRect(rect());
    }

    // Set up world space
    p.translate(width() / 2.0, height() / 2.0);
    p.scale(width(), -height());
    p.rotate(_rotate * 90);
    p.scale(1.0 / Field_Dimensions::Current_Dimensions.FloorLength(),
            1.0 / Field_Dimensions::Current_Dimensions.FloorWidth());

    // Set text rotation for world space
    _textRotation = -_rotate * 90;

    if (showCoords) {
        drawCoords(p);
    }

    // Get the latest LogFrame
    const std::shared_ptr<LogFrame> frame = currentFrame();

    if (!frame) {
        // No data available yet
        return;
    }

    // Make coordinate transformations
    _screenToWorld = Geometry2d::TransformMatrix();
    _screenToWorld *= Geometry2d::TransformMatrix::scale(
        Field_Dimensions::Current_Dimensions.FloorLength(),
        Field_Dimensions::Current_Dimensions.FloorWidth());
    _screenToWorld *=
        Geometry2d::TransformMatrix::rotate(-_rotate * M_PI / 2.0);
    _screenToWorld *=
        Geometry2d::TransformMatrix::scale(1.0 / width(), -1.0 / height());
    _screenToWorld *=
        Geometry2d::TransformMatrix::translate(-width() / 2.0, -height() / 2.0);

    _worldToTeam = Geometry2d::TransformMatrix();
    _worldToTeam *= Geometry2d::TransformMatrix::translate(
        0, Field_Dimensions::Current_Dimensions.Length() / 2.0f);
    if (frame->defend_plus_x()) {
        _worldToTeam *= Geometry2d::TransformMatrix::rotate(-M_PI / 2.0);
    } else {
        _worldToTeam *= Geometry2d::TransformMatrix::rotate(M_PI / 2.0);
    }

    _teamToWorld = Geometry2d::TransformMatrix();
    if (frame->defend_plus_x()) {
        _teamToWorld *= Geometry2d::TransformMatrix::rotate(M_PI / 2.0);
    } else {
        _teamToWorld *= Geometry2d::TransformMatrix::rotate(-M_PI / 2.0);
    }
    _teamToWorld *= Geometry2d::TransformMatrix::translate(
        0, -Field_Dimensions::Current_Dimensions.Length() / 2.0f);

    // Draw world-space graphics
    drawWorldSpace(p);

    // Everything after this point is drawn in team space.
    // Transform that into world space depending on defending goal.
    if (frame->defend_plus_x()) {
        p.rotate(90);
    } else {
        p.rotate(-90);
    }
    p.translate(0, -Field_Dimensions::Current_Dimensions.Length() / 2.0f);

    // Text has to be rotated so it is always upright on screen
    _textRotation = -_rotate * 90 + (frame->defend_plus_x() ? -90 : 90);

    drawTeamSpace(p);
}

void FieldView::drawWorldSpace(QPainter& p) {
    // Get the latest LogFrame
    const LogFrame* frame = _history->at(0).get();

    // Draw the field
    drawField(p, frame);

    // maps robots to their comet trails, so we can draw a path of where each
    // robot has been over the past X frames the pair used as a key is of the
    // form (team, robot_id).  Blue team = 1, yellow = 2. we only draw trails
    // for robots that exist in the current frame
    map<pair<int, int>, QPainterPath> cometTrails;

    /// populate @cometTrails with the past locations of each robot
    int pastLocationCount = 50;  // number of past locations to show
    const float prev_loc_scale = 0.4;
    for (int i = 0; i < pastLocationCount + 1 && i < _history->size(); i++) {
        const LogFrame* oldFrame = _history->at(i).get();
        if (oldFrame) {
            for (const SSL_WrapperPacket& wrapper : oldFrame->raw_vision()) {
                if (!wrapper.has_detection()) {
                    // useless
                    continue;
                }

                const SSL_DetectionFrame& detect = wrapper.detection();

                for (const SSL_DetectionRobot& r : detect.robots_blue()) {
                    pair<int, int> key(1, r.robot_id());
                    if (cometTrails.find(key) != cometTrails.end() || i == 0) {
                        QPointF pt(r.x() / 1000, r.y() / 1000);
                        if (i == 0)
                            cometTrails[key].moveTo(pt);
                        else
                            cometTrails[key].lineTo(pt);
                    }
                }

                for (const SSL_DetectionRobot& r : detect.robots_yellow()) {
                    pair<int, int> key(2, r.robot_id());
                    if (cometTrails.find(key) != cometTrails.end() || i == 0) {
                        QPointF pt(r.x() / 1000, r.y() / 1000);
                        if (i == 0)
                            cometTrails[key].moveTo(pt);
                        else
                            cometTrails[key].lineTo(pt);
                    }
                }
            }
        }
    }

    // draw robot comet trails
    const float cometTrailPenSize = 0.05;
    for (auto& kv : cometTrails) {
        QColor color = kv.first.first == 1 ? Qt::blue : Qt::yellow;
        QPen pen(color, cometTrailPenSize);
        pen.setCapStyle(Qt::RoundCap);
        p.setPen(pen);
        p.drawPath(kv.second);
    }

    // Raw vision
    if (showRawBalls || showRawRobots) {
        tempPen.setColor(QColor(0xcc, 0xcc, 0xcc));
        p.setPen(tempPen);
        for (const SSL_WrapperPacket& wrapper : frame->raw_vision()) {
            if (!wrapper.has_detection()) {
                // Useless
                continue;
            }

            const SSL_DetectionFrame& detect = wrapper.detection();

            if (showRawRobots) {
                for (const SSL_DetectionRobot& r : detect.robots_blue()) {
                    QPointF pos(r.x() / 1000, r.y() / 1000);
                    drawRobot(p, true, r.robot_id(), pos, r.orientation());
                    // p.drawEllipse(QPointF(r.x() / 1000, r.y() / 1000),
                    // Robot_Radius, Robot_Radius);
                }

                for (const SSL_DetectionRobot& r : detect.robots_yellow()) {
                    QPointF pos(r.x() / 1000, r.y() / 1000);
                    drawRobot(p, false, r.robot_id(), pos, r.orientation());
                    // p.drawEllipse(QPointF(r.x() / 1000, r.y() / 1000),
                    // Robot_Radius, Robot_Radius);
                }
            }

            if (showRawBalls) {
                for (const SSL_DetectionBall& b : detect.balls()) {
                    p.drawEllipse(QPointF(b.x() / 1000, b.y() / 1000),
                                  Ball_Radius, Ball_Radius);
                }
            }
        }
    }
}

void FieldView::drawTeamSpace(QPainter& p) {
    // Get the latest LogFrame
    const LogFrame* frame = _history->at(0).get();

    if (showTeamNames) {
        // Draw Team Names
        QFont savedFont = p.font();
        QFont fontstyle = p.font();
        fontstyle.setPointSize(20);
        p.setFont(fontstyle);
        p.setPen(bluePen);
        drawText(p, QPointF(0, 4.75), QString(frame->team_name_blue().c_str()),
                 true);  // Blue
        p.setPen(yellowPen);
        drawText(p, QPointF(0, 1.75),
                 QString(frame->team_name_yellow().c_str()), true);  // Yellow
        p.setFont(savedFont);
    }

    // Block off half the field
    if (!frame->use_our_half()) {
        const float FX = Field_Dimensions::Current_Dimensions.FloorWidth() / 2;
        const float FY1 = -Field_Dimensions::Current_Dimensions.Border();
        const float FY2 = Field_Dimensions::Current_Dimensions.Length() / 2;
        p.fillRect(QRectF(QPointF(-FX, FY1), QPointF(FX, FY2)),
                   QColor(0, 0, 0, 128));
    }
    if (!frame->use_opponent_half()) {
        const float FX = Field_Dimensions::Current_Dimensions.FloorWidth() / 2;
        const float FY1 = Field_Dimensions::Current_Dimensions.Length() / 2;
        const float FY2 = Field_Dimensions::Current_Dimensions.Length() +
                          Field_Dimensions::Current_Dimensions.Border();
        p.fillRect(QRectF(QPointF(-FX, FY1), QPointF(FX, FY2)),
                   QColor(0, 0, 0, 128));
    }

    if (showCoords) {
        drawCoords(p);
    }

    // History
    p.setBrush(Qt::NoBrush);
    QPainterPath ballTrail;
    for (unsigned int i = 0; i < 200 && i < _history->size(); ++i) {
        const LogFrame* oldFrame = _history->at(i).get();
        if (oldFrame && oldFrame->has_ball()) {
            QPointF pos = qpointf(oldFrame->ball().pos());

            if (i == 0)
                ballTrail.moveTo(pos);
            else
                ballTrail.lineTo(pos);
        }
    }
    QPen ballTrailPen(ballColor, 0.03);
    ballTrailPen.setCapStyle(Qt::RoundCap);
    p.setPen(ballTrailPen);
    p.drawPath(ballTrail);

    // Debug lines
    for (const DebugPath& path : frame->debug_paths()) {
        if (path.layer() < 0 || layerVisible(path.layer())) {
            tempPen.setColor(qcolor(path.color()));
            p.setPen(tempPen);
            std::vector<QPointF> pts;
            for (int i = 0; i < path.points_size(); ++i) {
                pts.push_back(qpointf(path.points(i)));
            }
            p.drawPolyline(pts.data(), pts.size());
        }
    }

    // Debug circles
    for (const DebugCircle& c : frame->debug_circles()) {
        if (c.layer() < 0 || layerVisible(c.layer())) {
            tempPen.setColor(c.color());
            p.setPen(tempPen);
            p.drawEllipse(qpointf(c.center()), c.radius(), c.radius());
        }
    }

    // Debug arcs
    for (const DebugArc& a : frame->debug_arcs()) {
        if (a.layer() < 0 || layerVisible(a.layer())) {
            tempPen.setColor(a.color());
            p.setPen(tempPen);

            auto c = a.center();
            auto t1 = a.start();
            auto t2 = a.end();
            auto R = a.radius();

            QRectF rect;
            rect.setX(-R + c.x());
            rect.setY(-R + c.y());
            rect.setWidth(R * 2);
            rect.setHeight(R * 2);

            t1 *= -(180 / M_PI) * 16;
            t2 *= -(180 / M_PI) * 16;

            p.drawArc(rect, t1, t2 - t1);
        }
    }

    // Debug text
    for (const DebugText& text : frame->debug_texts()) {
        if (text.layer() < 0 || layerVisible(text.layer())) {
            tempPen.setColor(text.color());
            p.setPen(tempPen);
            drawText(p, qpointf(text.pos()),
                     QString::fromStdString(text.text()), text.center());
        }
    }

    // Debug polygons
    p.setPen(Qt::NoPen);
    for (const DebugPath& path : frame->debug_polygons()) {
        if (path.layer() < 0 || layerVisible(path.layer())) {
            if (path.points_size() < 3) {
                fprintf(stderr, "Ignoring DebugPolygon with %d points\n",
                        path.points_size());
                continue;
            }

            QColor color = qcolor(path.color());
            color.setAlpha(64);
            p.setBrush(color);
            std::vector<QPointF> pts;
            for (int i = 0; i < path.points_size(); ++i) {
                pts.push_back(qpointf(path.points(i)));
            }
            p.drawConvexPolygon(pts.data(), pts.size());
        }
    }
    p.setBrush(Qt::NoBrush);

    // Text positioning vectors
    QPointF rtX = qpointf(Geometry2d::Point(0, 1).rotated(-_rotate * 90));
    QPointF rtY = qpointf(Geometry2d::Point(-1, 0).rotated(-_rotate * 90));

    // Opponent robots
    for (const LogFrame::Robot& r : frame->opp()) {
        drawRobot(p, !frame->blue_team(), r.shell(), qpointf(r.pos()),
                  r.angle(), r.ball_sense_status() == HasBall);
    }

    // Our robots
    int manualID = frame->manual_id();
    for (const LogFrame::Robot& r : frame->self()) {
        QPointF center = qpointf(r.pos());

        bool faulty = false;
        if (r.has_ball_sense_status() && (r.ball_sense_status() == Dazzled ||
                                          r.ball_sense_status() == Failed)) {
            faulty = true;
        }
        if (r.has_kicker_works() && !r.kicker_works()) {
            // 			faulty = true;
        }
        for (int i = 0; i < r.motor_status().size(); ++i) {
            if (r.motor_status(i) != Good) {
                faulty = true;
            }
        }
        if (r.has_battery_voltage() && r.battery_voltage() <= 14.3f) {
            faulty = true;
        }

        drawRobot(p, frame->blue_team(), r.shell(), center, r.angle(),
                  r.ball_sense_status() == HasBall, faulty);

        // Highlight the manually controlled robot
        if (manualID == r.shell()) {
            p.setPen(greenPen);
            const float r = Robot_Radius + .05;
            p.drawEllipse(center, r, r);
        }

        // Robot text
        QPointF textPos = center - rtX * 0.2 - rtY * (Robot_Radius + 0.1);
        for (const DebugText& text : r.text()) {
            if (text.layer() < 0 || layerVisible(text.layer())) {
                tempPen.setColor(text.color());
                p.setPen(tempPen);
                drawText(p, textPos, QString::fromStdString(text.text()),
                         false);
                textPos -= rtY * 0.1;
            }
        }
    }

    // Current ball position and velocity
    if (frame->has_ball()) {
        QPointF pos = qpointf(frame->ball().pos());
        QPointF vel = qpointf(frame->ball().vel());

        p.setPen(ballPen);
        p.setBrush(ballColor);
        p.drawEllipse(QRectF(-Ball_Radius + pos.x(), -Ball_Radius + pos.y(),
                             Ball_Diameter, Ball_Diameter));

        if (!vel.isNull()) {
            p.drawLine(pos, QPointF(pos.x() + vel.x(), pos.y() + vel.y()));
        }
    }
}

void FieldView::drawText(QPainter& p, QPointF pos, QString text, bool center) {
    p.save();
    p.translate(pos);
    p.rotate(_textRotation);
    p.scale(0.008, -0.008);

    if (center) {
        int flags = Qt::AlignHCenter | Qt::AlignVCenter;
        QRectF r = p.boundingRect(QRectF(), flags, text);
        p.drawText(r, flags, text);
    } else {
        p.drawText(QPointF(), text);
    }

    p.restore();
}

void FieldView::drawCoords(QPainter& p) {
    p.setPen(grayPen);

    // X
    p.drawLine(QPointF(0, 0), QPointF(0.25, 0));
    p.drawLine(QPointF(0.25, 0), QPointF(0.20, -0.05));
    p.drawLine(QPointF(0.25, 0), QPointF(0.20, 0.05));
    drawText(p, QPointF(0.25, 0.1), "+X");

    // Y
    p.drawLine(QPointF(0, 0), QPointF(0, 0.25));
    p.drawLine(QPointF(0, 0.25), QPointF(-0.05, 0.20));
    p.drawLine(QPointF(0, 0.25), QPointF(0.05, 0.20));
    drawText(p, QPointF(0.1, 0.25), "+Y");
}

void FieldView::drawField(QPainter& p, const LogFrame* frame) {
    p.save();

    // reset to center
    p.translate(-Field_Dimensions::Current_Dimensions.FloorLength() / 2.0,
                -Field_Dimensions::Current_Dimensions.FloorWidth() / 2.0);

    p.translate(Field_Dimensions::Current_Dimensions.Border(),
                Field_Dimensions::Current_Dimensions.Border());

    p.setPen(QPen(Qt::white, Field_Dimensions::Current_Dimensions.LineWidth() *
                                 2.0));  // double-width pen for visibility
                                         //(although its less accurate)
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(0, 0, Field_Dimensions::Current_Dimensions.Length(),
                      Field_Dimensions::Current_Dimensions.Width()));

    // set brush alpha to 0
    p.setBrush(QColor(0, 130, 0, 0));

    // reset to center
    p.translate(Field_Dimensions::Current_Dimensions.Length() / 2.0,
                Field_Dimensions::Current_Dimensions.Width() / 2.0);

    // centerline
    p.drawLine(QLineF(0, Field_Dimensions::Current_Dimensions.Width() / 2, 0,
                      -Field_Dimensions::Current_Dimensions.Width() / 2.0));

    // center circle
    p.drawEllipse(
        QRectF(-Field_Dimensions::Current_Dimensions.CenterRadius(),
               -Field_Dimensions::Current_Dimensions.CenterRadius(),
               Field_Dimensions::Current_Dimensions.CenterDiameter(),
               Field_Dimensions::Current_Dimensions.CenterDiameter()));

    p.translate(-Field_Dimensions::Current_Dimensions.Length() / 2.0, 0);

    // goal areas
    p.drawArc(QRectF(-Field_Dimensions::Current_Dimensions.ArcRadius(),
                     -Field_Dimensions::Current_Dimensions.ArcRadius() +
                         Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
                     2.f * Field_Dimensions::Current_Dimensions.ArcRadius(),
                     2.f * Field_Dimensions::Current_Dimensions.ArcRadius()),
              -90 * 16, 90 * 16);
    p.drawArc(QRectF(-Field_Dimensions::Current_Dimensions.ArcRadius(),
                     -Field_Dimensions::Current_Dimensions.ArcRadius() -
                         Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
                     2.f * Field_Dimensions::Current_Dimensions.ArcRadius(),
                     2.f * Field_Dimensions::Current_Dimensions.ArcRadius()),
              90 * 16, -90 * 16);
    p.drawLine(QLineF(Field_Dimensions::Current_Dimensions.ArcRadius(),
                      -Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
                      Field_Dimensions::Current_Dimensions.ArcRadius(),
                      Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f));
    // Penalty Mark
    p.drawEllipse(
        QRectF(-Field_Dimensions::Current_Dimensions.PenaltyDiam() / 2.0f +
                   Field_Dimensions::Current_Dimensions.PenaltyDist(),
               -Field_Dimensions::Current_Dimensions.PenaltyDiam() / 2.0f,
               Field_Dimensions::Current_Dimensions.PenaltyDiam(),
               Field_Dimensions::Current_Dimensions.PenaltyDiam()));

    p.translate(Field_Dimensions::Current_Dimensions.Length(), 0);

    p.drawArc(QRectF(-Field_Dimensions::Current_Dimensions.ArcRadius(),
                     -Field_Dimensions::Current_Dimensions.ArcRadius() +
                         Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
                     2.f * Field_Dimensions::Current_Dimensions.ArcRadius(),
                     2.f * Field_Dimensions::Current_Dimensions.ArcRadius()),
              -90 * 16, -90 * 16);
    p.drawArc(QRectF(-Field_Dimensions::Current_Dimensions.ArcRadius(),
                     -Field_Dimensions::Current_Dimensions.ArcRadius() -
                         Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
                     2.f * Field_Dimensions::Current_Dimensions.ArcRadius(),
                     2.f * Field_Dimensions::Current_Dimensions.ArcRadius()),
              90 * 16, 90 * 16);
    p.drawLine(QLineF(-Field_Dimensions::Current_Dimensions.ArcRadius(),
                      -Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
                      -Field_Dimensions::Current_Dimensions.ArcRadius(),
                      Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f));
    // Penalty Mark
    p.drawEllipse(
        QRectF(-Field_Dimensions::Current_Dimensions.PenaltyDiam() / 2.0f -
                   Field_Dimensions::Current_Dimensions.PenaltyDist(),
               -Field_Dimensions::Current_Dimensions.PenaltyDiam() / 2.0f,
               Field_Dimensions::Current_Dimensions.PenaltyDiam(),
               Field_Dimensions::Current_Dimensions.PenaltyDiam()));

    // goals
    float x[2] = {0, Field_Dimensions::Current_Dimensions.GoalDepth()};
    float y[2] = {Field_Dimensions::Current_Dimensions.GoalWidth() / 2.0f,
                  -Field_Dimensions::Current_Dimensions.GoalWidth() / 2.0f};

    bool flip = frame->blue_team() ^ frame->defend_plus_x();

    QColor goalColor = flip ? Qt::yellow : Qt::blue;
    p.setPen(
        QPen(goalColor,
             Field_Dimensions::Current_Dimensions.LineWidth() *
                 2.0));  // double-width for visibility, not real-life accuracy
    p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
    p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
    p.drawLine(QLineF(x[1], y[1], x[1], y[0]));

    x[0] -= Field_Dimensions::Current_Dimensions.Length();
    x[1] -= Field_Dimensions::Current_Dimensions.Length() +
            2 * Field_Dimensions::Current_Dimensions.GoalDepth();

    goalColor = flip ? Qt::blue : Qt::yellow;
    p.setPen(QPen(goalColor,
                  Field_Dimensions::Current_Dimensions.LineWidth() * 2.0));
    p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
    p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
    p.drawLine(QLineF(x[1], y[1], x[1], y[0]));

    p.restore();
}

void FieldView::drawRobot(QPainter& painter, bool blueRobot, int ID,
                          QPointF pos, float theta, bool hasBall, bool faulty) {
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::NoBrush);

    painter.save();

    painter.translate(pos.x(), pos.y());

    if (faulty) {
        painter.save();
        painter.setPen(redPen);
        painter.setBrush(QBrush{Qt::red, Qt::SolidPattern});
        auto r = Robot_Radius + 0.025;
        painter.drawEllipse(QPointF(0, 0), r, r);
        painter.restore();
    }

    if (blueRobot) {
        painter.setPen(bluePen);
        painter.setBrush(Qt::blue);
    } else {
        painter.setPen(yellowPen);
        painter.setBrush(Qt::yellow);
    }

    painter.rotate(theta * RadiansToDegrees + 90);

    int span = 40;

    int start = span * 16 + 90 * 16;
    int end = 360 * 16 - (span * 2) * 16;
    const float r = Robot_Radius;
    painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);

    if (showDotPatterns) {
        painter.setPen(Qt::NoPen);
        for (int i = 0; i < 4; i++) {
            painter.setBrush(QBrush(Dot_Pattern_Colors[ID][i]));
            QPointF center;
            center.setX((i >= 2) ? Dots_Small_Offset : Dots_Large_Offset);
            center.setX(center.x() * ((i == 1 || i == 2) ? -1 : 1));
            center.setY((i <= 1) ? Dots_Small_Offset : Dots_Large_Offset);
            center.setY(center.y() * ((i <= 1) ? -1 : 1));
            painter.drawEllipse(center, Dots_Radius, Dots_Radius);
        }
    }

    if (hasBall) {
        painter.setPen(redPen);
        const float r = Robot_Radius * 0.75f;
        painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);
    }

    painter.restore();

    // draw shell number
    painter.save();
    painter.translate(pos.x(), pos.y());
    if (blueRobot) {
        painter.setPen(whitePen);
    } else {
        painter.setPen(blackPen);
    }
    drawText(painter, QPointF(), QString::number(ID));
    painter.restore();
}

void FieldView::resizeEvent(QResizeEvent* e) {
    int givenW = e->size().width();
    int givenH = e->size().height();
    int needW, needH;
    if (_rotate & 1) {
        needH =
            roundf(givenW * Field_Dimensions::Current_Dimensions.FloorLength() /
                   Field_Dimensions::Current_Dimensions.FloorWidth());
        needW =
            roundf(givenH * Field_Dimensions::Current_Dimensions.FloorWidth() /
                   Field_Dimensions::Current_Dimensions.FloorLength());
    } else {
        needH =
            roundf(givenW * Field_Dimensions::Current_Dimensions.FloorWidth() /
                   Field_Dimensions::Current_Dimensions.FloorLength());
        needW =
            roundf(givenH * Field_Dimensions::Current_Dimensions.FloorLength() /
                   Field_Dimensions::Current_Dimensions.FloorWidth());
    }

    QSize size;
    if (needW < givenW) {
        size = QSize(needW, givenH);
    } else {
        size = QSize(givenW, needH);
    }

    if (size != e->size()) {
        resize(size);
    }
    e->accept();
}
