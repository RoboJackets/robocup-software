// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <iostream>
#include "MotionModule.hpp"

#include <QMouseEvent>
#include <boost/foreach.hpp>

using namespace std;
using namespace Motion;
using namespace Packet;

MotionModule::MotionModule(SystemState *state, const ConfigFile::MotionModule& cfg) :
	Module("Motion"), _guiInitialized(false), _config(cfg)
{
	_state = state;
	_configWidget = new QWidget();
	_ui.setupUi(_configWidget);

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

#if 0
void MotionModule::fieldOverlay(QPainter& p, Packet::LogFrame& lf) const
{
	for(unsigned int i=0; i<5; i++)
	{
		// draw the RRT: NOT IMPLEMENTED
		if (_ui.drawRRT->isChecked())
		{
			_robots[i]->drawRRT(p);
		}

		// draw the path: includes RRT
		if (_ui.drawPath->isChecked())
		{
			_robots[i]->drawPath(p);
		}

		// draw the trajectory from bezier curves
		if (_ui.drawBezierTraj->isChecked())
		{
			_robots[i]->drawBezierTraj(p);
		}

		// draw the control points from bezier curves
		if (_ui.drawBezierControl->isChecked())
		{
			_robots[i]->drawBezierControl(p);
		}

		// draw trajectory history
		if (_ui.drawHistory->isChecked())
		{
			_robots[i]->drawPoseHistory(p);
		}
	}
}
#endif

#if 0
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
			_state->self[_selectedRobotId].cmd.face = MotionCmd::Endpoint;
			_state->self[_selectedRobotId].cmd.goalOrientation = pos;
		}
	}
}
#endif

void MotionModule::run()
{
    BOOST_FOREACH(Robot* r, _robots)
    {
		r->proc();
    }
}
