
#include <FieldView.hpp>

#include <stdio.h>

#include <Network.hpp>
#include <LogUtils.hpp>
#include <Constants.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>

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

static const QPen redPen(Qt::red, 0);
static const QPen bluePen(Qt::blue, 0);
static const QPen yellowPen(Qt::yellow, 0);
static const QPen blackPen(Qt::black, 0);
static const QPen whitePen(Qt::white, 0);
static const QPen greenPen(Qt::green, 0);
static const QPen grayPen(Qt::gray, 0);

static QPen tempPen(Qt::white, 0);

static const QColor ballColor(0xff, 0x90, 0);
static const QPen ballPen(ballColor, 0);

FieldView::FieldView(QWidget* parent) :
	FieldBackgroundView(parent)
{
	showRawRobots = false;
	showRawBalls = false;
	showCoords = false;
    showDotPatterns = false;
    showTeamNames = false;
	_history = 0;
}

std::shared_ptr<LogFrame> FieldView::currentFrame()
{
	if (_history && !_history->empty())
	{
		return _history->at(0);
	} else {
		return std::shared_ptr<LogFrame>();
	}
}

void FieldView::layerVisible(int i, bool value)
{
    if (i >= 0 && i < _layerVisible.size())
    {
        _layerVisible[i] = value;
    }
    else if (i >= _layerVisible.size())
    {
        _layerVisible.resize(i+1);
        _layerVisible[i] = value;
    }
}

bool FieldView::layerVisible(int i) const
{
    if (i < _layerVisible.size())
    {
        return _layerVisible[i];
    } else {
        return false;
    }
}

void FieldView::paintEvent(QPaintEvent* e)
{
    // Get the latest LogFrame
    const std::shared_ptr<LogFrame> frame = currentFrame();
    if (!frame)
    {
        // No data available yet
        return;
    }

    // Set FieldBackgroundView settings
    setBlueTeam(frame->blue_team());
    setDefendingPlusX(frame->defend_plus_x());

    // Call superclass method to draw field
    FieldBackgroundView::paintEvent(e);

	QPainter p(this);

	//	antialiasing drastically improves rendering quality
	p.setRenderHint(QPainter::Antialiasing);

	if (!live)
	{
		// Non-live border
		p.setPen(QPen(Qt::red, 4));
		p.drawRect(rect());
	}

    // Make sure coordinate transforms are valid
    recalculateTransformsIfNeeded();

	// Transform coordinate system
    setupWorldSpace(&p);

	if (showCoords)
	{
		drawCoords(p, true);
	}


	// Draw world-space graphics
	drawWorldSpace(p);

	// Everything after this point is drawn in team space.
	// Transform that into world space depending on defending goal.
    if (defendingPlusX())
	{
		p.rotate(90);
	} else {
		p.rotate(-90);
	}
	p.translate(0, -fieldDimensions().Length() / 2.0f);

	drawTeamSpace(p);
}

void FieldView::drawWorldSpace(QPainter& p)
{
	// Get the latest LogFrame
	const LogFrame *frame = _history->at(0).get();

	// Raw vision
	if (showRawBalls || showRawRobots)
	{
		tempPen.setColor(QColor(0xcc, 0xcc, 0xcc));
		p.setPen(tempPen);
		for (const SSL_WrapperPacket& wrapper :  frame->raw_vision())
		{
			if (!wrapper.has_detection())
			{
				// Useless
				continue;
			}

			const SSL_DetectionFrame &detect = wrapper.detection();

			if (showRawRobots)
			{
				for (const SSL_DetectionRobot& r :  detect.robots_blue())
				{
					QPointF pos(r.x() / 1000, r.y() / 1000);
					// drawRobot(p, true, r.robot_id(), pos, r.orientation());
                    p.drawEllipse(QPointF(r.x() / 1000, r.y() / 1000), Robot_Radius, Robot_Radius);
				}

				for (const SSL_DetectionRobot& r :  detect.robots_yellow())
				{
					QPointF pos(r.x() / 1000, r.y() / 1000);
					// drawRobot(p, false, r.robot_id(), pos, r.orientation());
                    p.drawEllipse(QPointF(r.x() / 1000, r.y() / 1000), Robot_Radius, Robot_Radius);
				}
			}

			if (showRawBalls)
			{
				for (const SSL_DetectionBall& b :  detect.balls())
				{
					p.drawEllipse(QPointF(b.x() / 1000, b.y() / 1000), Ball_Radius, Ball_Radius);
				}
			}
		}
	}
}

void FieldView::drawTeamSpace(QPainter& p)
{
	// Get the latest LogFrame
	const LogFrame *frame = _history->at(0).get();

	if(showTeamNames)
    {
    	//Draw Team Names
    	QFont savedFont = p.font();
		QFont fontstyle = p.font();
		fontstyle.setPointSize(20);
		p.setFont(fontstyle);
		p.setPen(bluePen);
		drawText(p,QPointF(0,4.75), QString(frame->team_name_blue().c_str()), true, false); //Blue
		p.setPen(yellowPen);
		drawText(p,QPointF(0,1.75), QString(frame->team_name_yellow().c_str()), true, false); //Yellow
    	p.setFont(savedFont);
    }

	// Block off half the field
	if (!frame->use_our_half())
	{
		const float FX = fieldDimensions().FloorWidth() / 2;
		const float FY1 = -fieldDimensions().Border();
		const float FY2 = fieldDimensions().Length() / 2;
		p.fillRect(QRectF(QPointF(-FX, FY1), QPointF(FX, FY2)), QColor(0, 0, 0, 128));
	}
	if (!frame->use_opponent_half())
	{
		const float FX = fieldDimensions().FloorWidth() / 2;
		const float FY1 = fieldDimensions().Length() / 2;
		const float FY2 = fieldDimensions().Length() + fieldDimensions().Border();
		p.fillRect(QRectF(QPointF(-FX, FY1), QPointF(FX, FY2)), QColor(0, 0, 0, 128));
	}

	if (showCoords)
	{
		drawCoords(p, false);
	}

	// History
	p.setBrush(Qt::NoBrush);
	QPainterPath ballTrail;
	for (unsigned int i = 0; i < 200 && i < _history->size(); ++i)
	{
		const LogFrame *oldFrame = _history->at(i).get();
		if (oldFrame && oldFrame->has_ball()) {
			QPointF pos = qpointf(oldFrame->ball().pos());

			if (i == 0) ballTrail.moveTo(pos);
			else ballTrail.lineTo(pos);
		}
	}
	QPen ballTrailPen(ballColor, 0.03);
	ballTrailPen.setCapStyle(Qt::RoundCap);
	p.setPen(ballTrailPen);
	p.drawPath(ballTrail);

	// Debug lines
	for (const DebugPath& path :  frame->debug_paths())
	{
		if (path.layer() < 0 || layerVisible(path.layer()))
		{
			tempPen.setColor(qcolor(path.color()));
			p.setPen(tempPen);
			std::vector<QPointF> pts;
			for (int i = 0; i < path.points_size(); ++i)
			{
				pts.push_back( qpointf(path.points(i)) );
			}
			p.drawPolyline(pts.data(), pts.size());
		}
	}

	// Debug circles
	for (const DebugCircle& c :  frame->debug_circles())
	{
		if (c.layer() < 0 || layerVisible(c.layer()))
		{
			tempPen.setColor(c.color());
			p.setPen(tempPen);
			p.drawEllipse(qpointf(c.center()), c.radius(), c.radius());
		}
	}

	// Debug text
	for (const DebugText& text :  frame->debug_texts())
	{
		if (text.layer() < 0 || layerVisible(text.layer()))
		{
			tempPen.setColor(text.color());
			p.setPen(tempPen);
			drawText(p, qpointf(text.pos()), QString::fromStdString(text.text()), text.center(), false);
		}
	}

	// Debug polygons
	p.setPen(Qt::NoPen);
	for (const DebugPath& path :  frame->debug_polygons())
	{
		if (path.layer() < 0 || layerVisible(path.layer()))
		{
			if (path.points_size() < 3)
			{
				fprintf(stderr, "Ignoring DebugPolygon with %d points\n", path.points_size());
				continue;
			}

			QColor color = qcolor(path.color());
			color.setAlpha(64);
			p.setBrush(color);
			std::vector<QPointF> pts;
			for (int i = 0; i < path.points_size(); ++i)
			{
				pts.push_back( qpointf(path.points(i)) );
			}
			p.drawConvexPolygon(pts.data(), pts.size());
		}
	}
	p.setBrush(Qt::NoBrush);


    //  maps robots to their comet trails, so we can draw a path of where each robot has been over the past X frames
    //  the pair used as a key is of the form (our_team?, robot_id).
    //  we only draw trails for robots that exist in the current frame
    map<pair<bool, int>, QPainterPath> cometTrails;

    //  populate @cometTrails with the past locations of each robot
    int pastLocationCount = 50; //  number of past locations to show
    bool prev_defend_plus_x;
    const float prev_loc_scale = 0.4;
    for (int i = 0; i < pastLocationCount + 1 && i < _history->size(); i++) {
        const LogFrame *oldFrame = _history->at(i).get();
        if (oldFrame) {
            // If we switch from defending +x to -x, cut off comet trail b/c it won't make sense
            if (i == 0) prev_defend_plus_x = oldFrame->defend_plus_x();
            else if (prev_defend_plus_x != oldFrame->defend_plus_x()) break;
            // Their robots
            for (const LogFrame::Robot& r : oldFrame->opp()) {
                pair<int, int> key(false, r.shell());
                if (cometTrails.find(key) != cometTrails.end() || i == 0) {
                    if (i == 0) cometTrails[key].moveTo(qpointf(r.pos()));
                    else cometTrails[key].lineTo(qpointf(r.pos()));
                }
            }

            //  Our robots
            for (const LogFrame::Robot& r : oldFrame->self()) {
                pair<int, int> key(true, r.shell());
                if (cometTrails.find(key) != cometTrails.end() || i == 0) {
                    if (i == 0) cometTrails[key].moveTo(qpointf(r.pos()));
                    else cometTrails[key].lineTo(qpointf(r.pos()));
                }
            }


        }
    }

    //  draw robot comet trails
    const float cometTrailPenSize = 0.05;
    const QColor ourColor(isBlueTeam() ? Qt::blue : Qt::yellow);
    const QColor theirColor(isBlueTeam() ? Qt::yellow : Qt::blue);
    for (auto &kv : cometTrails) {
        QColor color = kv.first.first ? ourColor : theirColor;
        QPen pen(color, cometTrailPenSize);
        pen.setCapStyle(Qt::RoundCap);
        p.setPen(pen);
        p.drawPath(kv.second);
    }


	// Text positioning vectors
	QPointF rtX = qpointf(Geometry2d::Point(0, 1).rotated(-static_cast<int>(fieldOrientation()) * 90));
	QPointF rtY = qpointf(Geometry2d::Point(-1, 0).rotated(-static_cast<int>(fieldOrientation()) * 90));

	// Opponent robots
	for (const LogFrame::Robot &r :  frame->opp())
	{
		drawRobot(p, !frame->blue_team(), r.shell(), qpointf(r.pos()), r.angle(), r.ball_sense_status() == HasBall);
	}

	// Our robots
	int manualID = frame->manual_id();
	for (const LogFrame::Robot &r :  frame->self())
	{
		QPointF center = qpointf(r.pos());

		bool faulty = false;
		if (r.has_ball_sense_status() && (r.ball_sense_status() == Dazzled || r.ball_sense_status() == Failed))
		{
			faulty = true;
		}
		if (r.has_kicker_works() && !r.kicker_works())
		{
// 			faulty = true;
		}
		for (int i = 0; i < r.motor_status().size(); ++i)
		{
			if (r.motor_status(i) != Good)
			{
				faulty = true;
			}
		}
		if (r.has_battery_voltage() && r.battery_voltage() <= 14.3f)
		{
			faulty = true;
		}

		drawRobot(p, frame->blue_team(), r.shell(), center, r.angle(), r.ball_sense_status() == HasBall, faulty);

		// Highlight the manually controlled robot
		if (manualID == r.shell())
		{
			p.setPen(greenPen);
			const float r = Robot_Radius + .05;
			p.drawEllipse(center, r, r);
		}

		// Robot text
		QPointF textPos = center - rtX * 0.2 - rtY * (Robot_Radius + 0.1);
		for (const DebugText& text :  r.text())
		{
			if (text.layer() < 0 || layerVisible(text.layer()))
			{
				tempPen.setColor(text.color());
				p.setPen(tempPen);
				drawText(p, textPos, QString::fromStdString(text.text()), false, false);
				textPos -= rtY * 0.1;
			}
		}
	}

	// Current ball position and velocity
	if (frame->has_ball())
	{
		QPointF pos = qpointf(frame->ball().pos());
		QPointF vel = qpointf(frame->ball().vel());

		p.setPen(ballPen);
		p.setBrush(ballColor);
		p.drawEllipse(QRectF(-Ball_Radius + pos.x(), -Ball_Radius + pos.y(),
				Ball_Diameter, Ball_Diameter));

		if (!vel.isNull())
		{
			p.drawLine(pos, QPointF(pos.x() + vel.x(), pos.y() + vel.y()));
		}
	}
}

void FieldView::drawText(QPainter &p, QPointF pos, QString text, bool center, bool worldSpace)
{
	p.save();
	p.translate(pos);
	p.rotate(worldSpace ? textRotationForWorldSpace() : textRotationForTeamSpace());
	p.scale(0.008, -0.008);

	if (center)
	{
		int flags = Qt::AlignHCenter | Qt::AlignVCenter;
		QRectF r = p.boundingRect(QRectF(), flags, text);
		p.drawText(r, flags, text);
	} else {
		p.drawText(QPointF(), text);
	}

	p.restore();
}

// Called in world space
void FieldView::drawCoords(QPainter& p, bool worldSpace)
{
	p.setPen(grayPen);

	// X
	p.drawLine(QPointF(0, 0), QPointF(0.25, 0));
	p.drawLine(QPointF(0.25, 0), QPointF(0.20, -0.05));
	p.drawLine(QPointF(0.25, 0), QPointF(0.20, 0.05));
	drawText(p, QPointF(0.25, 0.1), "+X", true, worldSpace);

	// Y
	p.drawLine(QPointF(0, 0), QPointF(0, 0.25));
	p.drawLine(QPointF(0, 0.25), QPointF(-0.05, 0.20));
	p.drawLine(QPointF(0, 0.25), QPointF(0.05, 0.20));
	drawText(p, QPointF(0.1, 0.25), "+Y", true, worldSpace);
}

void FieldView::drawRobot(QPainter& painter, bool blueRobot, int ID, QPointF pos, float theta, bool hasBall, bool faulty)
{
	painter.setPen(Qt::NoPen);
	painter.setBrush(Qt::NoBrush);

	painter.save();

	painter.translate(pos.x(), pos.y());

	if (faulty)
	{
		painter.setPen(redPen);
	} else if (blueRobot)
	{
		painter.setPen(bluePen);
		painter.setBrush(Qt::blue);
	} else {
		painter.setPen(yellowPen);
		painter.setBrush(Qt::yellow);
	}

	painter.rotate(theta * RadiansToDegrees + 90);

    // Draw body shape
	const int span = 40;
	const int start = span*16 + 90*16;
	const int end = 360*16 - (span*2)*16;
	const float r = Robot_Radius;
	painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);

    if(showDotPatterns)
    {
        painter.setPen(Qt::NoPen);
        for(int i = 0; i < 4; i++)
        {
            painter.setBrush(QBrush(Dot_Pattern_Colors[ID][i]));
            QPointF center;
            center.setX( (i >= 2) ? Dots_Small_Offset : Dots_Large_Offset );
            center.setX( center.x() * ( (i == 1 || i == 2) ? -1 : 1 ) );
            center.setY( (i <= 1) ? Dots_Small_Offset : Dots_Large_Offset );
            center.setY( center.y() * ( (i <= 1) ? -1 : 1 ) );
            painter.drawEllipse(center, Dots_Radius, Dots_Radius);
        }
    }

	if (hasBall)
	{
		painter.setPen(redPen);
		const float r = Robot_Radius * 0.75f;
		painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);
	}

	painter.restore();

	//	draw shell number
	painter.save();
	painter.translate(pos.x(), pos.y());
	if (blueRobot) {
		painter.setPen(whitePen);
	} else {
		painter.setPen(blackPen);
	}
	drawText(painter, QPointF(), QString::number(ID), true, false);
	painter.restore();
}
