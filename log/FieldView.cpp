// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#include "FieldView.hpp"

#include <Constants.hpp>
#include <QPainter>
#include <QResizeEvent>

using namespace Constants;
using namespace Log;

FieldView::FieldView(QWidget* parent) :
	QGLWidget(parent), _team(UnknownTeam)
{
	_frame = 0;

	_tx = -Field::Length/2.0f;
	_ty = 0;
	_ta = -90;
	
	//turn on mouse tracking for modules that may need the event
	this->setMouseTracking(true);
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

void FieldView::addModule(Module* module)
{
	_modules.append(module);
}

Geometry::Point2d FieldView::toTeamSpace(int x, int y) const
{
	//meters per pixel
	float mpp = Constants::Floor::Length / width();
	
	float wx = x * mpp - Constants::Floor::Length/2.0f;
	float wy = Constants::Floor::Width/2.0 - y * mpp;
	
	Geometry::Point2d world(wx, wy);
	
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
	
	world.rotate(Geometry::Point2d(0,0), angle);
	world.y += Constants::Field::Length/2.0f;
	
	return world;
}

void FieldView::mousePressEvent(QMouseEvent* me)
{
	Q_FOREACH(Module* m, _modules)
	{
		m->mousePress(me, toTeamSpace(me->x(), me->y()));
	}
}

void FieldView::mouseReleaseEvent(QMouseEvent* me)
{
	Q_FOREACH(Module* m, _modules)
	{
		m->mouseRelease(me, toTeamSpace(me->x(), me->y()));
	}
}

void FieldView::mouseMoveEvent(QMouseEvent* me)
{
	Q_FOREACH(Module* m, _modules)
	{
		m->mouseMove(me, toTeamSpace(me->x(), me->y()));
	}
}

void FieldView::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	//painter.setRenderHint(QPainter::SmoothPixmapTransform);
	painter.scale(width()/Floor::Length, -height()/Floor::Width);

	// world space
	painter.translate(Floor::Length/2.0, -Floor::Width/2.0);

	drawField(painter);

	////team space
	painter.translate(_tx, _ty);
	painter.rotate(_ta);
	
	if (_frame)
	{
		//TODO handle display options...
		
		//draw the raw frame info
		//TODO draw only the last frame?? - Roman
		
		Q_FOREACH(const Module* m, _modules)
		{
			painter.save();
			m->fieldOverlay(painter, *_frame);
			painter.restore();
		}
		
#if 0
		painter.setPen(Qt::gray);
		BOOST_FOREACH(const Geometry::Segment &seg, _frame->rrt)
		{
			painter.drawLine(QPointF(seg.pt[0].x, seg.pt[0].y), QPointF(seg.pt[1].x, seg.pt[1].y));
		}
		
		painter.setPen(Qt::red);
		bool first = true;
		Geometry::Point2d last;
		BOOST_FOREACH(const Geometry::Point2d &pt, _frame->pathTest)
		{
			if (!first)
			{
				painter.drawLine(QPointF(pt.x, pt.y), QPointF(last.x, last.y));
			}
			first = false;
			last = pt;
		}
#endif
	}
}

void FieldView::drawField(QPainter& p)
{
	p.save();
	
	//reset to center
	p.translate(-Floor::Length/2.0, -Floor::Width/2.0);
	
	p.setPen(Qt::transparent);
	p.setBrush(QColor(0,85,0));
	p.drawRect(QRectF(0, 0, Floor::Length, Floor::Width));		
	
	p.translate(Field::Border, Field::Border);
	
	p.setPen(Qt::white);
	p.setBrush(QColor(0,130,0));
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
