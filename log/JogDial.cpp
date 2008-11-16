#include "JogDial.hpp"

#include <QMouseEvent>
#include <math.h>
#include <QPainter>

JogDial::JogDial(QWidget* parent) :
	QWidget(parent)
{
	_offset = 0;
	_spacing = 5.0f;
	_viewSpan = 180.0f;
}

void JogDial::paintEvent(QPaintEvent* pe)
{
	QPainter p(this);
	
	const float s = _offset - _spacing * (int)(_offset/_spacing);
	
	for (float angle = s - _viewSpan/2.0; angle <= s + _viewSpan/2.0 ; angle += _spacing)
	{
		int pos = (int)(sin(angle * M_PI/180.0f) * width()/2.0 + width()/2.0);
		
		p.drawLine(pos, 0, pos, height());
	}
	
	p.drawLine(0, 0, width(), 0);
	p.drawLine(0, height()-1, width(), height()-1);
}

void JogDial::mousePressEvent(QMouseEvent* me)
{
	_x = me->x();
}

void JogDial::mouseMoveEvent(QMouseEvent* me)
{
	int x = me->x();
	
	int diff = x - _x;
	_offset = diff * _viewSpan/width();
	
	update();
	valueChanged(offset());
}

void JogDial::mouseReleaseEvent(QMouseEvent* me)
{
	_offset = 0;
	update();
	valueChanged(offset());
}
