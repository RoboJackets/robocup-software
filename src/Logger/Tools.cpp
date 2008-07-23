#include "Tools.hpp"

#include <QGridLayout>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>

#include <Packet/SimCmd.hpp>
#include <Geometry/Point2d.hpp>
#include <Sizes.h>

using namespace Geometry;

#include <math.h>

Tools::Tools(Team t, Packet::VisionData& vd, QWidget* parent) :
	QWidget(parent), _sender(t), _vision(vd)
{
	QGridLayout* layout = new QGridLayout(this);

	QLabel* ridLabel = new QLabel("Robot:");
	layout->addWidget(ridLabel, 0, 0);

	QComboBox* robotCombo = new QComboBox();
	connect(robotCombo, SIGNAL(currentIndexChanged(int)), SLOT(changeRobot(int)));
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		robotCombo->addItem(QString::number(i));
	}

	layout->addWidget(robotCombo, 0 ,1);

	_none = new QPushButton("None");
	_none->setAutoExclusive(true);
	_none->setCheckable(true);
	_none->setChecked(true);
	layout->addWidget(_none, 1, 0);
    connect(_none, SIGNAL(clicked()), SLOT(stopSending()));

	_gotoPos = new QPushButton("Goto Point");
	_gotoPos->setAutoExclusive(true);
	_gotoPos->setCheckable(true);
	layout->addWidget(_gotoPos, 1, 1);

	_facePos = new QPushButton("Face Point");
	_facePos->setAutoExclusive(true);
	_facePos->setCheckable(true);
	layout->addWidget(_facePos, 1, 2);
	
	QGroupBox* skillsWidget = new QGroupBox("Skills", this);
	layout->addWidget(skillsWidget, 2, 0, 1, 3);
	
	_noSkill = new QPushButton("No Skill");
	connect(_noSkill, SIGNAL(clicked()), SLOT(noSkill()));
	
	_gotoBall = new QPushButton("Goto Ball");
	connect(_gotoBall, SIGNAL(clicked()), SLOT(gotoBall()));
	
	_handleBall = new QPushButton("Handle Ball");
	connect(_handleBall, SIGNAL(clicked()), SLOT(handleBall()));
	
	QGridLayout* skillsLayout = new QGridLayout(skillsWidget);
	skillsLayout->addWidget(_noSkill, 0, 0);
	skillsLayout->addWidget(_gotoBall, 0, 1);
	skillsLayout->addWidget(_handleBall, 0, 2);
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		//TODO: init angle and pos to current
		_pos[i] = Geometry::Point2d(0,0);
		_face[i] = Geometry::Point2d(0,0);
	}
    
    connect(&_timer, SIGNAL(timeout()), SLOT(send()));
}

Tools::~Tools()
{
}

void Tools::newPosition(float x, float y, float wx, float wy, QMouseEvent me)
{
	//mouse released at x,y team space
	if (me.button() == Qt::LeftButton && !_none->isChecked())
	{
		if (_gotoPos->isChecked())
		{
			_pos[_rid].x = x;
			_pos[_rid].y = y;
		}
		else if (_facePos->isChecked())
		{
			_face[_rid].x = x;
			_face[_rid].y = y;
		}

        if (!_timer.isActive())
        {
            printf("start\n");
            _timer.start(50);
        }
        send();
	}
	else if (me.button() == Qt::LeftButton)
	{
        Packet::SimCmd simCmd;
		simCmd.setBallPos = true;
		simCmd.ballPos = Geometry::Point2d(wx, wy);
		simCmd.ballVel = Point2d(0,0);
		
		_sender.send(simCmd);
	}
	else if (me.button() == Qt::RightButton)
	{
		if (_vision.ball.valid)
		{
			Point2d world(wx, wy);
			
			//turn ball position into world position
			Point2d worldBall(_vision.ball.pos.y - FIELD_LENGTH/2.0f, -_vision.ball.pos.x);
			
			Point2d dir = world - worldBall;
			dir = dir.norm() * 5.0; //shoot speed
			
            Packet::SimCmd simCmd;
			simCmd.ballVel = dir;
			
			//TODO be able to disable
			_sender.send(simCmd);
		}
	}
}

void Tools::changeRobot(int rid)
{
	_rid = rid;
}

void Tools::gotoBall()
{
	_lastSkill = Packet::SkillCmd::GotoBall;
    if (!_timer.isActive())
    {
        _timer.start(50);
    }
	sendSkill();
}

void Tools::handleBall()
{
//	_lastSkill = Packet::SkillCmd::HandleBall;
    if (!_timer.isActive())
    {
        _timer.start(50);
    }
	sendSkill();
}

void Tools::noSkill()
{
	_lastSkill = Packet::SkillCmd::None;
    _timer.stop();
	sendSkill();
}

void Tools::sendSkill()
{
	Packet::SkillCmd cmd;
	Packet::SkillCmd::Robot& rCmd = cmd.robots[_rid];
	
	rCmd.valid = true;
	rCmd.skill = _lastSkill;
	
	rCmd.motion.pos = _pos[_rid];
	rCmd.motion.face = _face[_rid];

	_sender.send(cmd);
}

void Tools::send()
{
    sendSkill();
    
    Packet::MotionCmd cmd;
    cmd.robots[_rid].valid = true;

    cmd.robots[_rid].pos = _pos[_rid];
    cmd.robots[_rid].face = _face[_rid];
    
    _sender.send(cmd);
}

void Tools::stopSending()
{
    _timer.stop();
}
