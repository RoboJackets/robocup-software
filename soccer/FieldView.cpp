// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#include "FieldView.hpp"

#include "draw.hpp"
#include <Constants.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <protobuf/SimCommand.pb.h>
#include <Network/Network.hpp>

#include <QPainter>
#include <QResizeEvent>
#include <boost/foreach.hpp>

using namespace boost;
using namespace Constants;

// Converts from meters to m/s for manually shooting the ball
static const float ShootScale = 5;

FieldView::FieldView(QWidget* parent) :
	QWidget(parent),
	_team(UnknownTeam),
	_sender(Network::Address, Network::SimCommandPort)
{
	state = 0;
	_dragBall = false;
	_frame = 0;
	_showVision = true;

	_tx = -Field::Length/2.0f;
	_ty = 0;
	_ta = -90;
	
	//turn on mouse tracking for modules that may need the event
	setMouseTracking(true);
	setAutoFillBackground(false);
	
	_updateTimer.setSingleShot(true);
	connect(&_updateTimer, SIGNAL(timeout()), SLOT(update()));
}

void FieldView::team(Team t)
{
	_team = t;

	if (_team == Blue)
	{
		_tx = Field::Length/2.0f;
		_ta = 90;
	}
}

void FieldView::frame(Packet::LogFrame* frame)
{
	_frame = frame;
	update();
}

void FieldView::addModule(shared_ptr<Module> module)
{
	_modules.append(module);
}

Geometry2d::Point FieldView::toTeamSpace(int x, int y) const
{
	//meters per pixel
	float mpp = Constants::Floor::Length / width();
	
	float wx = x * mpp - Constants::Floor::Length/2.0f;
	float wy = Constants::Floor::Width/2.0 - y * mpp;
	
	Geometry2d::Point world(wx, wy);
	
	//return world coords if team is unknown
	if (_team == UnknownTeam)
	{
		return world;
	}
	
	//default angle for blue
	float angle = -90;
	if (_team == Yellow)
	{
		angle = 90;
	}
	
	world.rotate(Geometry2d::Point(0,0), angle);
	world.y += Constants::Field::Length/2.0f;
	
	return world;
}

Geometry2d::Point FieldView::toWorldSpace(Geometry2d::Point pt, bool translate) const
{
	float offset = translate ? Constants::Field::Length / 2 : 0;
	if (_team == Blue)
	{
		return Geometry2d::Point(offset - pt.y, pt.x);
	} else if (_team == Yellow)
	{
		return Geometry2d::Point(pt.y - offset, -pt.x);
	} else {
		return pt;
	}
}

void FieldView::mouseDoubleClickEvent(QMouseEvent* me)
{
	Packet::SimCommand cmd;
	cmd.mutable_ball()->set_valid(true);
	cmd.mutable_ball()->mutable_pos()->CopyFrom(toWorldSpace(toTeamSpace(me->x(), me->y())));
	//FIXME - _sender.send(cmd);
}

void FieldView::mousePressEvent(QMouseEvent* me)
{
	Geometry2d::Point pos = toTeamSpace(me->x(), me->y());
	
	//look for a robot selection
	if (me->button() == Qt::RightButton && _frame)
	{
		//reset to no robot if we don't find anything
		int newID = -1;
		
		for (int i = 0; i < 5; ++i)
		{
			if (pos.distTo(_frame->self[i].pos) < Constants::Robot::Radius)
			{
				newID = i;
				break;
			}
		}
		
		if (state && newID != state->manualID)
		{
			state->manualID = newID;
			
			BOOST_FOREACH(shared_ptr<Module> m, _modules)
			{
				m->robotSelected(state->manualID);
			}
		}
		
		return;
	}
	
	if (me->button() == Qt::LeftButton && _frame && _frame->ball.valid && pos.nearPoint(_frame->ball.pos, Constants::Ball::Radius))
	{
		_dragBall = true;
		_dragTo = pos;
	}
	
	BOOST_FOREACH(shared_ptr<Module> m, _modules)
	{
		m->mousePress(me, pos);
	}
}

void FieldView::mouseReleaseEvent(QMouseEvent* me)
{
	if (_dragBall)
	{
		Packet::SimCommand cmd;
		cmd.mutable_ball()->set_valid(true);
		cmd.mutable_ball()->mutable_pos()->CopyFrom(toWorldSpace(_frame->ball.pos));
		cmd.mutable_ball()->mutable_vel()->CopyFrom(toWorldSpace((_frame->ball.pos - _dragTo) * ShootScale, false));
		//FIXME - _sender.send(cmd);
		
		_dragBall = false;
		update();
	} else {
		BOOST_FOREACH(shared_ptr<Module> m, _modules)
		{
			m->mouseRelease(me, toTeamSpace(me->x(), me->y()));
		}
	}
}

void FieldView::mouseMoveEvent(QMouseEvent* me)
{
	if (_dragBall)
	{
		_dragTo = toTeamSpace(me->x(), me->y());
		update();
	} else {
		BOOST_FOREACH(shared_ptr<Module> m, _modules)
		{
			m->mouseMove(me, toTeamSpace(me->x(), me->y()));
		}
	}
}

void FieldView::paintEvent(QPaintEvent* event)
{
	QPainter p(this);
	
	p.fillRect(rect(), QColor(0, 85.0, 0));
	
	float scale = 1;
	
	float scalex = width()/Floor::Length * scale;
	float scaley = height()/Floor::Width * scale;
	
	p.scale(scalex, -scaley);
	
	// world space
	p.translate(Floor::Length/2.0, -Floor::Width/2.0);
	
	drawField(p);

	if (!_frame || !state)
	{
		return;
	}
	
	////team space
	p.translate(_tx, _ty);
	p.rotate(_ta);
	
	if (_dragBall)
	{
		p.setPen(Qt::white);
		Geometry2d::Point ball = _frame->ball.pos;
		p.drawLine(ball.toQPointF(), _dragTo.toQPointF());
		
		if (ball != _dragTo)
		{
			p.setPen(Qt::gray);
			
			float speed = (ball - _dragTo).mag();
			Geometry2d::Point shoot = ball + (ball - _dragTo) / speed * 8;
			speed *= ShootScale;
			
			p.drawLine(ball.toQPointF(), shoot.toQPointF());
			p.save();
			p.translate(_dragTo.toQPointF());
			p.rotate((_team == Blue) ? -90 : 90);
			p.scale(0.008, -0.008);
			p.drawText(_dragTo.toQPointF(), QString("%1 m/s").arg(speed, 0, 'f', 1));
			p.restore();
		}
	}
	
	if (state->manualID != -1)
	{
		p.setPen(Qt::green);
		
		Geometry2d::Point center = _frame->self[state->manualID].pos;
		const float r = Constants::Robot::Radius + .05;
		p.drawEllipse(center.toQPointF(), r, r);
	}
	
	// draw debug lines
	BOOST_FOREACH(const Packet::LogFrame::DebugLine& seg, _frame->debugLines)
	{
		p.setPen(QColor(seg.color[0], seg.color[1], seg.color[2]));
		p.drawLine(seg.pt[0].toQPointF(), seg.pt[1].toQPointF());
	}

	// draw debug circles
	BOOST_FOREACH(const Packet::LogFrame::DebugCircle& cir, _frame->debugCircles)
	{
		p.setPen(QColor(cir.color[0], cir.color[1], cir.color[2]));
		p.drawEllipse(cir.center.toQPointF(), cir.radius(), cir.radius());
	}

	// draw debug text
	BOOST_FOREACH(const Packet::LogFrame::DebugText& tex, _frame->debugText)
	{
		p.setPen(QColor(tex.color[0], tex.color[1], tex.color[2]));
		QString text = QString::fromStdString(tex.text);
		p.save();
		p.translate(tex.pos.x, tex.pos.y);
		p.rotate((_frame->team == Blue) ? -90 : 90);
		p.scale(.008, -.008);
		QRect r = p.boundingRect(0, 0, 0, 0, 0, text);
		p.drawText(-r.width() / 2, r.height() / 2 - 4, text);
		p.restore();
	}

	p.setPen(Qt::NoPen);
	BOOST_FOREACH(const Packet::LogFrame::DebugPolygon &polygon, _frame->debugPolygons)
	{
		if (polygon.vertices.empty())
		{
			printf("Empty polygon\n");
			continue;
		}
		
		p.setBrush(QColor(polygon.color[0], polygon.color[1], polygon.color[2], 64));
		QPointF pts[polygon.vertices.size()];
		for (unsigned int i = 0; i < polygon.vertices.size(); ++i)
		{
			pts[i] = polygon.vertices[i].toQPointF();
		}
		p.drawConvexPolygon(pts, polygon.vertices.size());
	}
	p.setBrush(Qt::NoBrush);

	if (_showVision)
	{
		BOOST_FOREACH(const Packet::Vision& vision, _frame->rawVision)
		{
			if (!vision.sync)
			{
				/* don't draw the robots twice
				BOOST_FOREACH(const Packet::Vision::Robot& r, vision.blue)
				{
					drawRobot(p, Blue, r.shell, r.pos, r.angle, _frame->team);
				}

				BOOST_FOREACH(const Packet::Vision::Robot& r, vision.yellow)
				{
					drawRobot(p, Yellow, r.shell, r.pos, r.angle, _frame->team);
				}
				*/

				BOOST_FOREACH(const Packet::Vision::Ball& b, vision.balls)
				{
					//drawBall(p, b.pos);
					// draw the raw vision
					p.setPen(QColor(0xcc, 0xcc, 0xcc));
					p.drawEllipse(_frame->ball.pos.toQPointF(), Constants::Ball::Radius, Constants::Ball::Radius);
				}
			}
		}
	}
	BOOST_FOREACH(const Packet::LogFrame::Robot &r, _frame->self)
	{
		if (r.valid)
		{
			drawRobot(p, _frame->team, r.shell, r.pos, r.angle, _frame->team, r.haveBall);
		}
	}

	Team opp = opponentTeam(_frame->team);
	BOOST_FOREACH(const Packet::LogFrame::Robot &r, _frame->opp)
	{
		if (r.valid)
		{
			drawRobot(p, opp, r.shell, r.pos, r.angle, _frame->team, r.haveBall);
		}
	}

	if (_frame->ball.valid)
	{
		drawBall(p, _frame->ball.pos, _frame->ball.vel);
	}
	
	// Referee rules
	p.setPen(Qt::black);
	if (_frame->gameState.stayAwayFromBall() && _frame->ball.valid)
	{
		p.setBrush(Qt::NoBrush);
		p.drawEllipse(_frame->ball.pos.toQPointF(), Constants::Field::CenterRadius, Constants::Field::CenterRadius);
	}
	
	BOOST_FOREACH(shared_ptr<Module> m, _modules)
	{
		p.save();
		m->fieldOverlay(p, *_frame);
		p.restore();
	}
	
	p.end();
	_updateTimer.start(30);
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
	
	p.setPen(Qt::blue);
	p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
	p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
	p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
	
	x[0] -= Field::Length;
	x[1] -= Field::Length + 2 * Field::GoalDepth;
	
	p.setPen(Qt::yellow);
	p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
	p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
	p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
	
	p.restore();
}

void FieldView::resizeEvent(QResizeEvent* event)
{
	int w = event->size().width();
	int h = int(w * Constants::Floor::Aspect);

	if (h > event->size().height())
	{
		h = event->size().height();
		w = int(h/Constants::Floor::Aspect);
	}

	this->resize(w,h);
	event->accept();
}
