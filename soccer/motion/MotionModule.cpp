// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "MotionModule.hpp"

#include <QMouseEvent>
#include <boost/foreach.hpp>

using namespace Motion;
using namespace Packet;

MotionModule::MotionModule(SystemState *state, const ConfigFile::MotionModule& cfg) :
	Module("Motion"),
	_config(cfg)
{
	_state = state;
	_configWidget = new QWidget();
	ui.setupUi(_configWidget);

	//seems to be the only way to set the widgets parent to an object
	((QObject*)_configWidget)->setParent((QObject*)this);
	QMetaObject::connectSlotsByName(this);

	//initialize empty robots
    for(unsigned int i=0 ; i<5 ; i++)
    {
        _robots[i] = new Robot(cfg.robot, i);
		_robots[i]->setSystemState(_state);
	}
}

MotionModule::~MotionModule()
{
    for (unsigned int i=0; i<5; i++)
    {
        delete _robots[i];
        _robots[i] = 0;
    }
}

QWidget* MotionModule::widget() const
{
	return _configWidget;
}

void MotionModule::fieldOverlay(QPainter& p, Packet::LogFrame& lf) const
{
	for(unsigned int i=0; i<5; i++)
	{
		if (ui.drawRRT->isChecked())
		{
			_robots[i]->drawRRT(p);
		}

		if (ui.drawPath->isChecked())
		{
			_robots[i]->drawPath(p);
		}
	}
}

void MotionModule::on_pos_kp_valueChanged(double value)
{
	for(unsigned int i=0; i<5; i++)
	{
		_robots[i]->setPosKp(value);
	}
}

void MotionModule::on_pos_ki_valueChanged(double value)
{
	for(unsigned int i=0; i<5; i++)
	{
		_robots[i]->setPosKi(value);
	}
}

void MotionModule::on_pos_kd_valueChanged(double value)
{
	for(unsigned int i=0; i<5; i++)
	{
		_robots[i]->setPosKd(value);
	}
}

void MotionModule::on_ang_kp_valueChanged(double value)
{
	for(unsigned int i=0; i<5; i++)
	{
		_robots[i]->setAngKp(value);
	}
}

void MotionModule::on_ang_ki_valueChanged(double value)
{
	for(unsigned int i=0; i<5; i++)
	{
		_robots[i]->setAngKi(value);
	}
}

void MotionModule::on_ang_kd_valueChanged(double value)
{
	for(unsigned int i=0; i<5; i++)
	{
		_robots[i]->setAngKd(value);
	}
}

void MotionModule::mousePress(QMouseEvent* me, Geometry2d::Point pos)
{
	if (_selectedRobotId != -1)
	{
		if (me->button() == Qt::LeftButton)
		{
			_state->self[_selectedRobotId].cmd.goalPosition = pos;
		}
		else if (me->button() == Qt::MidButton)
		{
			_state->self[_selectedRobotId].cmd.face = LogFrame::MotionCmd::Endpoint;
			_state->self[_selectedRobotId].cmd.goalOrientation = pos;
		}
	}
}

void MotionModule::run()
{
	BOOST_FOREACH(Robot* r, _robots)
    {
		r->proc();
    }
}

void MotionModule::slowRun()
{
	BOOST_FOREACH(Robot* r, _robots)
	{
		r->slow();
	}
}
