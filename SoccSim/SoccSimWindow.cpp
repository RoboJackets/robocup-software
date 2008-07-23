#include "SoccSimWindow.hpp"

SoccSimWindow::SoccSimWindow() :
	_vision(_env)
{
	_ui.setupUi(this);
	
	_env.start();
	_vision.start();
	
	_ui.ball->setChecked(true);
	_ui.y0->setChecked(true);
	
	viewer.setEnv(&_env);
	connect(_ui.displayButton, SIGNAL(toggled(bool)), &viewer, SLOT(setVisible(bool)));
	
	//_ui.displayButton->setChecked(true);
}

SoccSimWindow::~SoccSimWindow()
{
	
}

void SoccSimWindow::toggleRobot(Team t, unsigned int id, bool active)
{
	if (active)
	{
		_env.addRobot(t, id);
	}
	else
	{
		_env.removeRobot(t, id);
	}
}

void SoccSimWindow::on_ball_toggled(bool b)
{
	if (b)
	{
		_env.enableBall();
	}
	else
	{
		_env.disableBall();
	}
}

void SoccSimWindow::on_actionError_triggered(bool c)
{
	if (c)
	{
		_vision.enableError();
	}
	else
	{
		_vision.disableError();
	}
}

void SoccSimWindow::on_blueToggle_toggled(bool b)
{
	_ui.b0->setChecked(b);
	_ui.b1->setChecked(b);
	_ui.b2->setChecked(b);
	_ui.b3->setChecked(b);
	_ui.b4->setChecked(b);
}

void SoccSimWindow::on_yellowToggle_toggled(bool b)
{
	_ui.y0->setChecked(b);
	_ui.y1->setChecked(b);
	_ui.y2->setChecked(b);
	_ui.y3->setChecked(b);
	_ui.y4->setChecked(b);
}
