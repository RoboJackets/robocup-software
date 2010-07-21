// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <FieldView.hpp>
#include <FieldView.moc>

#include <stdio.h>

#include <Network.hpp>
#include <Logger.hpp>
#include <LogUtils.hpp>
#include <Constants.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>

#include <QLayout>
#include <QPainter>
#include <QResizeEvent>
#include <boost/foreach.hpp>
#include <algorithm>
#include <sys/socket.h>

using namespace std;
using namespace boost;
using namespace Constants;
using namespace Packet;

QColor ballColor(0xff, 0x90, 0);

// Converts from meters to m/s for manually shooting the ball
static const float ShootScale = 5;

FieldView::FieldView(QWidget* parent) :
	QWidget(parent)
{
	showRawRobots = false;
	showRawBalls = false;
	showCoords = false;
	logger = 0;
	_dragMode = DRAG_NONE;
	_rotate = 0;
	_frameNumber = -1;

	setAttribute(Qt::WA_OpaquePaintEvent);
}

void FieldView::rotate(int value)
{
	_rotate = value;
	
	// Fix size
	updateGeometry();
	
	update();
}

void FieldView::mouseDoubleClickEvent(QMouseEvent* me)
{
	if (me->button() == Qt::LeftButton)
	{
		placeBall(me->posF());
	}
}

void FieldView::mousePressEvent(QMouseEvent* me)
{
	Geometry2d::Point pos = _worldToTeam * _screenToWorld * me->posF();
	
	if (me->button() == Qt::LeftButton)
	{
		placeBall(me->posF());
		_dragMode = DRAG_PLACE;
	} else if (me->button() == Qt::RightButton)
	{
		if (_frame.has_ball() && pos.nearPoint(_frame.ball().pos(), Constants::Ball::Radius))
		{
			// Drag to shoot the ball
			_dragMode = DRAG_SHOOT;
			_dragTo = pos;
		} else {
			// Look for a robot selection
			int newID = -1;
			for (int i = 0; i < _frame.self_size(); ++i)
			{
				if (pos.distTo(_frame.self(i).pos()) < Constants::Robot::Radius)
				{
					newID = _frame.self(i).shell();
					break;
				}
			}
			
			if (newID != _frame.manual_id())
			{
				robotSelected(newID);
			}
		}
	}
}

void FieldView::mouseReleaseEvent(QMouseEvent* me)
{
	if (_dragMode == DRAG_SHOOT)
	{
		SimCommand cmd;
		_teamToWorld.transformDirection(_shot).set(cmd.mutable_ball_vel());
		sendSimCommand(cmd);
		
		update();
	}
	
	_dragMode = DRAG_NONE;
}

void FieldView::mouseMoveEvent(QMouseEvent* me)
{
	switch (_dragMode)
	{
		case DRAG_SHOOT:
			_dragTo = _worldToTeam * _screenToWorld * me->posF();
			break;
		
		case DRAG_PLACE:
			placeBall(me->posF());
			break;
		
		default:
			break;
	}
	update();
}

void FieldView::placeBall(QPointF pos)
{
	SimCommand cmd;
	cmd.mutable_ball_pos()->CopyFrom(_screenToWorld * pos);
	cmd.mutable_ball_vel()->set_x(0);
	cmd.mutable_ball_vel()->set_y(0);
	sendSimCommand(cmd);
}

void FieldView::sendSimCommand(const Packet::SimCommand& cmd)
{
	std::string out;
	cmd.SerializeToString(&out);
	_simCommandSocket.writeDatagram(&out[0], out.size(), QHostAddress(QHostAddress::LocalHost), SimCommandPort);
}

void FieldView::paintEvent(QPaintEvent* event)
{
	QPainter p(this);
	
	// Green background
	p.fillRect(rect(), QColor(0, 85.0, 0));
	
	// Set up world space
	p.translate(width() / 2.0, height() / 2.0);
	p.scale(width(), -height());
	p.rotate(_rotate * 90);
	p.scale(1.0 / Floor::Length, 1.0 / Floor::Width);
	
	// Set text rotation for world space
	_textRotation = -_rotate * 90;
	
	drawField(p);

	if (showCoords)
	{
		drawCoords(p);
	}
	
	if (!logger)
	{
		// Can't get frames without a logger
		return;
	}
	
	_frameNumber = logger->lastFrame();
	if (_frameNumber < 0)
	{
		// No data available yet
		return;
	}
	logger->getFrame(_frameNumber, _frame);
	
	// Make coordinate transformations
	_screenToWorld = Geometry2d::TransformMatrix();
	_screenToWorld *= Geometry2d::TransformMatrix::scale(Constants::Floor::Length, Constants::Floor::Width);
	_screenToWorld *= Geometry2d::TransformMatrix::rotate(-_rotate * 90);
	_screenToWorld *= Geometry2d::TransformMatrix::scale(1.0 / width(), -1.0 / height());
	_screenToWorld *= Geometry2d::TransformMatrix::translate(-width() / 2.0, -height() / 2.0);
	
	_worldToTeam = Geometry2d::TransformMatrix();
	_worldToTeam *= Geometry2d::TransformMatrix::translate(0, Constants::Field::Length / 2.0f);
	if (_frame.defend_plus_x())
	{
		_worldToTeam *= Geometry2d::TransformMatrix::rotate(-90);
	} else {
		_worldToTeam *= Geometry2d::TransformMatrix::rotate(90);
	}
	
	_teamToWorld = Geometry2d::TransformMatrix();
	if (_frame.defend_plus_x())
	{
		_teamToWorld *= Geometry2d::TransformMatrix::rotate(90);
	} else {
		_teamToWorld *= Geometry2d::TransformMatrix::rotate(-90);
	}
	_teamToWorld *= Geometry2d::TransformMatrix::translate(0, -Constants::Field::Length / 2.0f);

	// Check number of debug layers
	if (layerVisible.size() != _frame.debug_layers_size())
	{
		layerVisible.resize(_frame.debug_layers_size());
	}
	
	// Raw vision, drawn in world space
	if (showRawBalls || showRawRobots)
	{
		p.setPen(QColor(0xcc, 0xcc, 0xcc));
		BOOST_FOREACH(const SSL_WrapperPacket& wrapper, _frame.raw_vision())
		{
			if (!wrapper.has_detection())
			{
				// Useless
				continue;
			}
			
			const SSL_DetectionFrame &detect = wrapper.detection();
			
			if (showRawRobots)
			{
				BOOST_FOREACH(const SSL_DetectionRobot& r, detect.robots_blue())
				{
					p.drawEllipse(QPointF(r.x() / 1000, r.y() / 1000), Constants::Robot::Radius, Constants::Robot::Radius);
				}

				BOOST_FOREACH(const SSL_DetectionRobot& r, detect.robots_yellow())
				{
					p.drawEllipse(QPointF(r.x() / 1000, r.y() / 1000), Constants::Robot::Radius, Constants::Robot::Radius);
				}
			}
			
			if (showRawBalls)
			{
				BOOST_FOREACH(const SSL_DetectionBall& b, detect.balls())
				{
					p.drawEllipse(QPointF(b.x() / 1000, b.y() / 1000), Constants::Ball::Radius, Constants::Ball::Radius);
				}
			}
		}
	}
	
	// Everything after this point is drawn in team space.
	// Transform that back into world space depending on defending goal.
	if (_frame.defend_plus_x())
	{
		p.rotate(90);
	} else {
		p.rotate(-90);
	}
	p.translate(0, -Field::Length / 2.0f);

	// Text has to be rotated so it is always upright on screen
	_textRotation = -_rotate * 90 + (_frame.defend_plus_x() ? -90 : 90);
	
	if (showCoords)
	{
		drawCoords(p);
	}
	
	// History
	LogFrame oldFrame;
	for (int i = 0; i < 200 && logger->getFrame(_frameNumber - i, oldFrame); ++i)
	{
		if (oldFrame.has_ball())
		{
			QPointF pos = qpointf(oldFrame.ball().pos());
			
			p.setPen(Qt::NoPen);
			QColor c = ballColor;
			c.setAlpha(255 - i);
			p.setBrush(c);
			
			p.drawEllipse(QRectF(-Ball::Radius + pos.x(), -Ball::Radius + pos.y(),
					Ball::Diameter, Ball::Diameter));
		}
	}
	p.setBrush(Qt::NoBrush);
	
	// Simulator drag-to-shoot
	if (_dragMode == DRAG_SHOOT)
	{
		p.setPen(Qt::white);
		Geometry2d::Point ball = _frame.ball().pos();
		p.drawLine(ball.toQPointF(), _dragTo.toQPointF());
		
		if (ball != _dragTo)
		{
			p.setPen(Qt::gray);
			
			_shot = (ball - _dragTo) * ShootScale;
			float speed = _shot.mag();
			Geometry2d::Point shotExtension = ball + _shot / speed * 8;
			
			p.drawLine(ball.toQPointF(), shotExtension.toQPointF());
			drawText(p, _dragTo.toQPointF(), QString("%1 m/s").arg(speed, 0, 'f', 1));
		}
	}
	
	// Debug lines
	BOOST_FOREACH(const DebugPath& path, _frame.debug_paths())
	{
		if (path.layer() < 0 || layerVisible[path.layer()])
		{
			p.setPen(qcolor(path.color()));
			QPointF pts[path.points_size()];
			for (int i = 0; i < path.points_size(); ++i)
			{
				pts[i] = qpointf(path.points(i));
			}
			p.drawPolyline(pts, path.points_size());
		}
	}

	// Debug circles
	BOOST_FOREACH(const DebugCircle& c, _frame.debug_circles())
	{
		if (c.layer() < 0 || layerVisible[c.layer()])
		{
			p.setPen(qcolor(c.color()));
			p.drawEllipse(qpointf(c.center()), c.radius(), c.radius());
		}
	}

	// Debug text
	BOOST_FOREACH(const DebugText& text, _frame.debug_texts())
	{
		if (text.layer() < 0 || layerVisible[text.layer()])
		{
			p.setPen(qcolor(text.color()));
			drawText(p, qpointf(text.pos()), QString::fromStdString(text.text()), text.center());
		}
	}

	// Debug polygons
	p.setPen(Qt::NoPen);
	BOOST_FOREACH(const DebugPath& path, _frame.debug_polygons())
	{
		if (path.layer() < 0 || layerVisible[path.layer()])
		{
			if (path.points_size() < 3)
			{
				fprintf(stderr, "Ignoring DebugPolygon with %d points\n", path.points_size());
				continue;
			}
			
			QColor color = qcolor(path.color());
			color.setAlpha(64);
			p.setBrush(color);
			QPointF pts[path.points_size()];
			for (int i = 0; i < path.points_size(); ++i)
			{
				pts[i] = qpointf(path.points(i));
			}
			p.drawConvexPolygon(pts, path.points_size());
		}
	}
	p.setBrush(Qt::NoBrush);

	// Filtered robot positions
	int manualID = _frame.manual_id();
	BOOST_FOREACH(const LogFrame::Robot &r, _frame.self())
	{
		QPointF center = qpointf(r.pos());
		drawRobot(p, _frame.blue_team(), r.shell(), center, r.angle(), r.has_ball());
		
		// Highlight the manually controlled robot
		if (manualID == r.shell())
		{
			p.setPen(Qt::green);
			const float r = Constants::Robot::Radius + .05;
			p.drawEllipse(center, r, r);
		}
	}

	BOOST_FOREACH(const LogFrame::Robot &r, _frame.opp())
	{
		drawRobot(p, !_frame.blue_team(), r.shell(), qpointf(r.pos()), r.angle(), r.has_ball());
	}

	// Current ball position and velocity
	if (_frame.has_ball())
	{
		QPointF pos = qpointf(_frame.ball().pos());
		QPointF vel = qpointf(_frame.ball().vel());
		
		p.setPen(ballColor);
		p.setBrush(ballColor);
		p.drawEllipse(QRectF(-Ball::Radius + pos.x(), -Ball::Radius + pos.y(),
				Ball::Diameter, Ball::Diameter));
		
		if (!vel.isNull())
		{
			p.drawLine(pos, QPointF(pos.x() + vel.x(), pos.y() + vel.y()));
		}
	}
}

void FieldView::drawText(QPainter &p, QPointF pos, QString text, bool center)
{
	p.save();
	p.translate(pos);
	p.rotate(_textRotation);
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

void FieldView::drawCoords(QPainter& p)
{
	p.setPen(Qt::gray);
	
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

void FieldView::drawField(QPainter& p)
{
	p.save();
	
	//reset to center
	p.translate(-Floor::Length/2.0, -Floor::Width/2.0);
	
	p.translate(Field::Border, Field::Border);
	
	p.setPen(Qt::white);
	p.setBrush(Qt::NoBrush);
	p.drawRect(QRectF(0, 0, Field::Length, Field::Width));
	
	//set brush alpha to 0
	p.setBrush(QColor(0,130,0, 0));
	
	//reset to center
	p.translate(Field::Length/2.0, Field::Width/2.0);
	
	//centerline
	p.drawLine(QLineF(0, Field::Width/2,0, -Field::Width/2.0));
	
	//center circle
	p.drawEllipse(QRectF(-Field::ArcRadius, -Field::ArcRadius, 
		Field::CenterDiameter, Field::CenterDiameter));
	
	p.translate(-Field::Length/2.0, 0);
	
	//goal areas
	p.drawArc(QRectF(-Field::ArcRadius, -Field::ArcRadius + .175, Field::CenterDiameter, Field::CenterDiameter), -90*16, 90*16);
	p.drawArc(QRectF(-Field::ArcRadius, -Field::ArcRadius - .175, Field::CenterDiameter, Field::CenterDiameter), 90*16, -90*16);
	p.drawLine(QLineF(Field::ArcRadius, -.175, Field::ArcRadius, .175));
	
	p.translate(Field::Length, 0);
	
	p.drawArc(QRectF(-Field::ArcRadius, -Field::ArcRadius + .175, Field::CenterDiameter, Field::CenterDiameter), -90*16, -90*16);
	p.drawArc(QRectF(-Field::ArcRadius, -Field::ArcRadius - .175, Field::CenterDiameter, Field::CenterDiameter), 90*16, 90*16);
	p.drawLine(QLineF(-Field::ArcRadius, -.175, -Field::ArcRadius, .175));
		
	// goals
	float x[2] = {0, Field::GoalDepth};
	float y[2] = {Field::GoalWidth/2.0, -Field::GoalWidth/2.0};
	
	bool flip = _frame.blue_team() ^ _frame.defend_plus_x();
	
	p.setPen(flip ? Qt::yellow : Qt::blue);
	p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
	p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
	p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
	
	x[0] -= Field::Length;
	x[1] -= Field::Length + 2 * Field::GoalDepth;
	
	p.setPen(flip ? Qt::blue : Qt::yellow);
	p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
	p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
	p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
	
	p.restore();
}

void FieldView::drawRobot(QPainter& painter, bool blueRobot, int ID, QPointF pos, float theta, bool hasBall)
{
	painter.setPen(Qt::white);
	painter.setBrush(Qt::NoBrush);
	
	painter.save();
	
	painter.translate(pos.x(), pos.y());
	
	drawText(painter, QPointF(), QString::number(ID));
	
	if (blueRobot)
	{
		painter.setPen(Qt::blue);
	} else {
		painter.setPen(Qt::yellow);
	}
	
	painter.rotate(theta+90);
	
	int span = 40;
	
	int start = span*16 + 90*16;
	int end = 360*16 - (span*2)*16;
	const float r = Constants::Robot::Radius;
	painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);
	
	if (hasBall)
	{
		painter.setPen(Qt::red);
		const float r = Constants::Robot::Radius * 0.75f;
		painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);
	}
	
	painter.restore();
}

void FieldView::resizeEvent(QResizeEvent* e)
{
	int givenW = e->size().width();
	int givenH = e->size().height();
	int needW, needH;
	if (_rotate & 1)
	{
		needH = roundf(givenW * Constants::Floor::Length / Constants::Floor::Width);
		needW = roundf(givenH * Constants::Floor::Width / Constants::Floor::Length);
	} else {
		needH = roundf(givenW * Constants::Floor::Width / Constants::Floor::Length);
		needW = roundf(givenH * Constants::Floor::Length / Constants::Floor::Width);
	}
	
	QSize size;
	if (needW < givenW)
	{
		size = QSize(needW, givenH);
	} else {
		size = QSize(givenW, needH);
	}
	
	if (size != e->size())
	{
		resize(size);
	}
    e->accept();
}
