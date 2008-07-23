#include "../Packet.hpp"
#include "LogMotion.hpp"

#include <QWidget>

using namespace Log;

LogMotionType LogMotionType::LogMotion::_type;

void LogMotionType::LogMotion::display(QPainter& p, Team t)
{
	p.setPen(Qt::black);
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		if (_data.robots[i].valid)
		{
			Packet::LogMotion::Robot r = _data.robots[i];
			
			QPointF curr(r.currPos.x, r.currPos.y);
			QPointF pdir(r.pdir.x, r.pdir.y);
			
			p.drawLine(curr, QPointF(r.destPos.x, r.destPos.y));
			
			p.drawLine(curr, curr + pdir * r.distRemaining);
		}
	}
}

QWidget* LogMotionType::configuration()
{
	QWidget* config = new QWidget();
	return config;
}
