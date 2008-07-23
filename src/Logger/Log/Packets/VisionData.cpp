#include "VisionData.hpp"

#include <QWidget>
#include <QLabel>
#include <QGridLayout>

#include <Sizes.h>
#include <Graphikos/Ball.hpp>
#include <Graphikos/Robot.hpp>

using namespace Log;
using namespace Graphikos;

Log::VisionDataType VisionDataType::VisionData::_type;

QWidget* VisionDataType::configuration()
{
	QWidget* widget = new QWidget();
	QLabel* test = new QLabel("test", widget);
	
	return widget;
}

QWidget* VisionDataType::information()
{
	_infoWidget = new QWidget();
	QGridLayout* l = new QGridLayout(_infoWidget);
	
	_timestamp = new QLabel();
	l->addWidget(_timestamp, 0, 0, 1, 5);
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_robots[i] = new QLabel("S. " + QString::number(i));
		_robots[i]->setStyleSheet("color: red");
		_robots[i]->setAlignment(Qt::AlignCenter);
		l->addWidget(_robots[i], 1, i);
		
		_robots[i+5] = new QLabel("O. " + QString::number(i));
		_robots[i+5]->setStyleSheet("color: red");
		_robots[i+5]->setAlignment(Qt::AlignCenter);
		l->addWidget(_robots[i+5], 2, i);
	}
	
	l->setRowStretch(3, 1);
	
	return _infoWidget;
}

void VisionDataType::VisionData::updateInformationWidget()
{
	_type._timestamp->setText("<b>Timestamp: </b>" + QString::number(_data.timestamp));

	for (unsigned int i=0 ; i<5 ; ++i)
	{
		if (_data.self[i].valid)
		{
			_type._robots[i]->setStyleSheet("color: green");
		}
		else
		{
			_type._robots[i]->setStyleSheet("color: red");	
		}
		
		if (_data.opp[i].valid)
		{
			_type._robots[i+5]->setStyleSheet("color: green");
		}
		else
		{
			_type._robots[i+5]->setStyleSheet("color: red");
		}
	}
}

void VisionDataType::VisionData::display(QPainter& p, Team t)
{
	p.setPen(Qt::black);
	
	if (_data.ball.valid)
	{
		Ball::paint(p, _data.ball.pos.x, _data.ball.pos.y);
	}
		
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		Packet::VisionData::Robot self = _data.self[i];
		if (self.valid)
		{
			Robot::paint(p, t, i, self.pos.x, self.pos.y, self.theta);
		}
		
		Packet::VisionData::Robot opp = _data.opp[i];
		if (opp.valid)
		{
			Robot::paint(p, opponentTeam(t), i, opp.pos.x, opp.pos.y, opp.theta);
		}
	}
}
