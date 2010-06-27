// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#include "FieldView.hpp"

#include <Constants.hpp>
#include <Point.hpp>
#include <Segment.hpp>
#include <SimCommand.hpp>
#include <Network/Network.hpp>

#include <QPainter>
#include <QResizeEvent>
#include <boost/foreach.hpp>

using namespace boost;
using namespace Constants;
using namespace Log;

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
	cmd.ball.valid = true;
	cmd.ball.pos = toWorldSpace(toTeamSpace(me->x(), me->y()));
	_sender.send(cmd);
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
		cmd.ball.valid = true;
		cmd.ball.pos = toWorldSpace(_frame->ball.pos);
		cmd.ball.vel = toWorldSpace((_frame->ball.pos - _dragTo) * ShootScale, false);
		_sender.send(cmd);
		
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
	QPainter painter(this);
	
	painter.fillRect(rect(), QColor(0, 85.0, 0));
	
	float scale = 1;
	
	float scalex = width()/Floor::Length * scale;
	float scaley = height()/Floor::Width * scale;
	
	painter.scale(scalex, -scaley);
	
	// world space
	painter.translate(Floor::Length/2.0, -Floor::Width/2.0);
	
	drawField(painter);

	////team space
	painter.translate(_tx, _ty);
	painter.rotate(_ta);
	
	if (_dragBall && _frame)
	{
		painter.setPen(Qt::white);
		Geometry2d::Point ball = _frame->ball.pos;
		painter.drawLine(ball.toQPointF(), _dragTo.toQPointF());
		
		if (ball != _dragTo)
		{
			painter.setPen(Qt::gray);
			
			float speed = (ball - _dragTo).mag();
			Geometry2d::Point shoot = ball + (ball - _dragTo) / speed * 8;
			speed *= ShootScale;
			
			painter.drawLine(ball.toQPointF(), shoot.toQPointF());
			painter.save();
			painter.translate(_dragTo.toQPointF());
			painter.rotate((_team == Blue) ? -90 : 90);
			painter.scale(0.008, -0.008);
			painter.drawText(_dragTo.toQPointF(), QString("%1 m/s").arg(speed, 0, 'f', 1));
			painter.restore();
		}
	}
	
	if (_frame)
	{
		//TODO handle display options...
		
		if (state && state->manualID != -1)
		{
			painter.setPen(Qt::green);
			
			Geometry2d::Point center = _frame->self[state->manualID].pos;
			const float r = Constants::Robot::Radius + .05;
			painter.drawEllipse(center.toQPointF(), r, r);
		}
		
		//draw the raw frame info
		//TODO draw only the last frame?? - Roman
		
		BOOST_FOREACH(shared_ptr<Module> m, _modules)
		{
			painter.save();
			m->fieldOverlay(painter, *_frame);
			painter.restore();
		}
	}
	
	painter.end();
	_updateTimer.start(30);
}

void FieldView::drawField(QPainter& p)
{
	p.save();
	
	//reset to center
	p.translate(-Floor::Length/2.0, -Floor::Width/2.0);
	
	p.setPen(Qt::NoPen);
	
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
