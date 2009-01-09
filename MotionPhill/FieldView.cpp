#include "FieldView.hpp"
#include "drawing/Elements.hpp"


#include <Constants.hpp>
#include <QPainter>
#include <QResizeEvent>

using namespace Constants;

FieldView::FieldView(Team t, QWidget* parent) :
	QWidget(parent), _team(t)
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
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			const Packet::LogFrame::Robot& s = _frame->self[i];
			const Packet::LogFrame::Robot& o = _frame->opp[i];

			if (s.valid)
			{
				drawRobot(painter, _team, s.shell, s.pos, s.angle);
			}

			if (o.valid)
			{
				drawRobot(painter, _team, o.shell, o.pos, o.angle);
			}
		}

		if (_frame->ball.valid)
		{
			drawBall(painter,_frame->ball.pos);
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
