#include "Forward.hpp"
#include "../Kick.hpp"
#include <Constants.hpp>

using namespace Geometry2d;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Forward> behavior("forward");

Gameplay::Behaviors::Forward::Forward(GameplayModule * gameplay, Role * role):
	Behavior(gameplay, role)
	{
	_kick = new Gameplay::Behaviors::Kick(gameplay);
	}

Gameplay::Behaviors::Forward::~Forward()
{
	delete _kick;
}

void Gameplay::Behaviors::Forward::run()
{
	//leave dribblers on
	robot()->dribble(20);

	//Copy in important state data
	const Packet::LogFrame::Ball& b = ball();
	Geometry2d::Point ball_pos = b.pos;

	//get teammate position
	Point tm_pos = _teammate->robot()->pos();
	Point tm_vel = _teammate->robot()->vel();

	//check what other team is doing
	bool teammate_has_ball = !_teammate->isIntercept();
	bool teammate_nearer_ball = robot()->pos().distTo(ball_pos) > tm_pos.distTo(ball_pos);
	if (teammate_has_ball)
	{
		//expect that the ball with go across the field
		bool ball_shot_on_left = ball_pos.x > 0;

		Point dest;
		if (ball_shot_on_left)
		{
			//go left
			dest = tm_pos + Point(-1.5,1);
		}
		else
		{
			//go right
			dest = tm_pos + Point(1.5,1);
		}
		robot()->move(dest);
		robot()->face(ball_pos);
	}
	else if (teammate_nearer_ball)
	{
		//back up along vector to ball
		Point dest = ball_pos + (robot()->pos() - ball_pos).normalized() * 0.5;
		robot()->move(dest);
		robot()->face(ball_pos);
	}
	else
	{
		//just run normal kick in auto mode
		_kick->run();
	}
}

void Gameplay::Behaviors::Forward::start()
{

	_kick->robot(robot());
	_kick->start();
	//find the other forward
	std::list<Robot*> teammates;
	_gameplay->find_by_type<Forward*>(teammates);
	if (teammates.size() > 0)
	{
		_teammate = dynamic_cast<Forward*>(teammates.front()->behavior());
	}
}

float Gameplay::Behaviors::Forward::score(Robot* robot)
{
	return Constants::Field::Length - robot->pos().y;
}

bool Gameplay::Behaviors::Forward::isIntercept()
{
	return _kick->isIntercept();
}
