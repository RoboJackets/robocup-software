#include "FieldView.hpp"

#include <Constants.hpp>
#include <QPainter>
#include <QResizeEvent>

#include "drawing/Elements.hpp"

using namespace Constants;

FieldView::FieldView(Team t, QWidget* parent) :
	QGLWidget(parent), _team(t)
{
	_frame = 0;
	
	_tx = -Field::Length/2.0f;
	_ty = 0;
	_ta = -90;
	
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

void FieldView::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	
	painter.setRenderHint(QPainter::Antialiasing);
	painter.scale(width()/Floor::Length, -height()/Floor::Width);
	
	// world space
	painter.translate(Floor::Length/2.0, -Floor::Width/2.0);
	
	drawField(painter);
	
	////team space
	painter.translate(_tx, _ty);
	painter.rotate(_ta);
	
	if (_frame)
	{
		Q_FOREACH(const Packet::LocVision::Robot& r, _frame->vision.self)
		{
			drawRobot(painter, _team, r.shell, r.pos, r.angle);
		}
		
		Q_FOREACH(const Packet::LocVision::Robot& r, _frame->vision.opp)
		{
			drawRobot(painter, opponentTeam(_team), r.shell, r.pos, r.angle);
		}
		
		Q_FOREACH(const Packet::LocVision::Ball& b, _frame->vision.balls)
		{
			drawBall(painter, b.pos);
		}
	}
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
